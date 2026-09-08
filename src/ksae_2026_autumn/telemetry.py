"""Read-only, frame-indexed telemetry. No CARLA or torch import at module scope."""

import json
import math
import statistics
import time
from collections import deque
from pathlib import Path

SCHEMA_VERSION = 1
CONTROL_FIELDS = (
    "throttle",
    "brake",
    "steer",
    "hand_brake",
    "reverse",
    "manual_gear_shift",
    "gear",
)


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def control_dict(control):
    return {key: getattr(control, key) for key in CONTROL_FIELDS}


def vector(value):
    return [float(value.x), float(value.y), float(value.z)]


def rotation(value):
    return [math.radians(value.roll), math.radians(value.pitch), math.radians(value.yaw)]


def actor_state(snapshot, actor):
    transform = snapshot.get_transform()
    velocity = vector(snapshot.get_velocity())
    box = actor.bounding_box
    return {
        "id": int(actor.id),
        "type_id": actor.type_id,
        "position_m": vector(transform.location),
        "rpy_rad": rotation(transform.rotation),
        "velocity_mps": velocity,
        "speed_mps": math.sqrt(sum(v * v for v in velocity)),
        "acceleration_mps2": vector(snapshot.get_acceleration()),
        "angular_velocity_radps": [
            math.radians(x) for x in vector(snapshot.get_angular_velocity())
        ],
        "bbox_center_local_m": vector(box.location),
        "bbox_extent_m": vector(box.extent),
        "bbox_rpy_local_rad": rotation(box.rotation),
    }


def describe(values):
    if not values:
        return {"count": 0, "median_ms": None, "p95_ms": None, "max_ms": None}
    ordered = sorted(values)
    return {
        "count": len(values),
        "median_ms": statistics.median(values),
        "p95_ms": ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)],
        "max_ms": max(values),
    }


class RouteTelemetry:
    """One route's lifecycle; a row is committed only after apply_control returns.

    Submitted control is RPC-call evidence, not an acknowledgement of the next
    physics state. Snapshot state describes the instant before that command.
    """

    def __init__(self, directory, metadata, enabled, world, ego):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = dict(metadata, schema_version=SCHEMA_VERSION, logging_enabled=enabled)
        self.enabled, self.world, self.ego = enabled, world, ego
        self.agent_calls = self.agent_returns = self.submit_calls = self.rows = 0
        self.pending = None
        self.closed = False
        self.errors = []
        self.agent_times = []
        self.active_agent_times = []
        self.logger_times = []
        self.first_frame = self.last_frame = None
        self.last_lidar_frame = None
        self.lidar_pairs = deque()
        self.stream = None
        write_json(self.directory / "route.json", self.metadata)
        if enabled:
            self.stream = (self.directory / "ticks.jsonl").open(
                "x", encoding="utf-8", buffering=65536
            )

    def call_agent(self, callback, agent, input_data, timestamp):
        if self.closed or self.pending is not None:
            raise RuntimeError(
                "Telemetry lifecycle error: closed route or unsubmitted previous action"
            )
        self.agent_calls += 1
        self.pending = {"capture_wall_ms": 0.0}
        capture_start = time.perf_counter()
        was_initialized = bool(getattr(agent, "initialized", False))
        try:
            if self.enabled:
                snapshot = self.world.get_snapshot()
                frame = int(snapshot.frame)
                ego_snapshot = snapshot.find(self.ego.id)
                if ego_snapshot is None:
                    raise RuntimeError("Ego is absent from the current WorldSnapshot")
                actors, unmatched = [], []
                for actor in self.world.get_actors():
                    if actor.id == self.ego.id or not actor.type_id.startswith(
                        ("vehicle.", "walker.pedestrian.")
                    ):
                        continue
                    state = snapshot.find(actor.id)
                    if state is None:
                        unmatched.append(int(actor.id))
                    else:
                        try:
                            actors.append(actor_state(state, actor))
                        except RuntimeError:
                            # Scenario threads may remove an NPC while its snapshot
                            # still exists. Report the missing metadata explicitly.
                            unmatched.append(int(actor.id))
                sensor_frames = {key: int(value[0]) for key, value in input_data.items()}
                self.pending.update(
                    {
                        "schema_version": SCHEMA_VERSION,
                        **{
                            key: self.metadata[key]
                            for key in ("run_id", "route_id", "repetition", "branch_id")
                        },
                        "frame": frame,
                        "state_frame": frame,
                        "decision_frame": frame,
                        "sim_time_s": float(snapshot.timestamp.elapsed_seconds),
                        "agent_time_s": float(timestamp),
                        "sensor_frames": sensor_frames,
                        "sensor_frame_offsets": {
                            key: frame - value for key, value in sensor_frames.items()
                        },
                        "ego": actor_state(ego_snapshot, self.ego),
                        "actors": sorted(actors, key=lambda item: item["id"]),
                        "unmatched_actor_ids": unmatched,
                        "previous_control_observed": control_dict(self.ego.get_control()),
                        "controller": "E2E",
                        "injected_delay_ticks": 0,
                    }
                )
            self.pending["capture_wall_ms"] = (time.perf_counter() - capture_start) * 1000
            start = time.perf_counter()
            # Exactly one call, including native initialization/early-return paths.
            control = callback(input_data, timestamp)
            elapsed = (time.perf_counter() - start) * 1000
            self.agent_returns += 1
            self.agent_times.append(elapsed)
            cfg = agent.config
            need = int(cfg.lidar_seq_len * cfg.data_save_freq)
            image_only = cfg.backbone == "aim"
            inference_ran = was_initialized and (image_only or len(agent.lidar_buffer) >= need)
            if inference_ran:
                self.active_agent_times.append(elapsed)
            if self.enabled:
                extra_start = time.perf_counter()
                current = self.pending["sensor_frames"].get("lidar")
                if was_initialized and not image_only and current is not None:
                    self.lidar_pairs.append([self.last_lidar_frame, current])
                    while len(self.lidar_pairs) > need:
                        self.lidar_pairs.popleft()
                self.last_lidar_frame = current
                history = []
                if inference_ran and not image_only:
                    for i in range(int(cfg.lidar_seq_len)):
                        history.append(self.lidar_pairs[-(i * int(cfg.data_save_freq) + 1)])
                sources = list(self.pending["sensor_frames"].values())
                sources.extend(x for pair in history for x in pair if x is not None)
                source_frame = max(self.pending["sensor_frames"].values())
                self.pending.update(
                    {
                        "agent_step_wall_ms": elapsed,
                        "inference_ran": inference_ran,
                        "warmup": not inference_ran,
                        "warmup_reason": "initialization"
                        if not was_initialized
                        else ("lidar_history" if not inference_ran else None),
                        "initial_control_delay_active": int(agent.step)
                        < int(cfg.inital_frames_delay),
                        "lidar_input_frame_pairs": history,
                        "input_frame_min": min(sources),
                        "input_frame_max": max(sources),
                        "action_source_frame": source_frame,
                        "action_observation_age_s": (self.pending["frame"] - source_frame) * 0.05,
                        "e2e_control": control_dict(control),
                    }
                )
                self.pending["capture_wall_ms"] += (time.perf_counter() - extra_start) * 1000
            return control
        except BaseException as error:
            self.errors.append(f"agent/capture: {type(error).__name__}: {error}")
            raise

    def submitted(self, control, frame):
        self.submit_calls += 1
        if self.pending is None:
            raise RuntimeError("apply_control was observed without a matching agent call")
        try:
            frame = int(frame)
            if self.first_frame is None:
                self.first_frame = frame
            self.last_frame = frame
            if self.enabled:
                start = time.perf_counter()
                if frame != self.pending["frame"]:
                    raise RuntimeError("World advanced between observation and control submission")
                self.pending["command_submit_frame"] = frame
                self.pending["selected_control"] = control_dict(control)
                self.pending["submitted_control"] = control_dict(control)
                self.stream.write(
                    json.dumps(self.pending, allow_nan=False, separators=(",", ":")) + "\n"
                )
                self.rows += 1
                if self.rows % 20 == 0:
                    self.stream.flush()
                self.logger_times.append(
                    self.pending["capture_wall_ms"] + (time.perf_counter() - start) * 1000
                )
            self.pending = None
        except BaseException as error:
            self.errors.append(f"submit/write: {type(error).__name__}: {error}")
            raise

    def close(self, reason):
        if self.closed:
            return
        self.closed = True
        if self.pending is not None:
            self.errors.append("An agent call has no committed control-submission record")
        try:
            if self.stream is not None:
                self.stream.close()
        except Exception as error:
            self.errors.append(f"close: {error}")
        write_json(
            self.directory / "summary.json",
            {
                "schema_version": SCHEMA_VERSION,
                "route_id": self.metadata["route_id"],
                "logging_enabled": self.enabled,
                "closed": True,
                "close_reason": reason,
                "agent_calls": self.agent_calls,
                "agent_returns": self.agent_returns,
                "submit_calls": self.submit_calls,
                "rows": self.rows,
                "first_frame": self.first_frame,
                "last_frame": self.last_frame,
                "errors": self.errors,
                "agent_step_all": describe(self.agent_times),
                "agent_step_active": describe(self.active_agent_times),
                "logger_capture_and_write": describe(self.logger_times),
            },
        )
