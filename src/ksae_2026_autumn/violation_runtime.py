"""CARLA observation adapter. No control changes and no simulator ticks here."""

import json
import math
import threading
import time
from pathlib import Path

from ksae_2026_autumn.route_corridor import from_route
from ksae_2026_autumn.telemetry import describe, write_json
from ksae_2026_autumn.violation_geometry import fixed_wheel_points, validate_bbox
from ksae_2026_autumn.violations import classify_wheels, collision_episodes, finalize_rows


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def transform_from_state(state):
    import carla

    roll, pitch, yaw = [math.degrees(v) for v in state["rpy_rad"]]
    return carla.Transform(carla.Location(*state["position_m"]), carla.Rotation(pitch, yaw, roll))


def wheel_points(ego, transform):
    """Compatibility helper for fixtures; never queries wheel physics coordinates."""
    profile = validate_bbox(ego)
    state = {
        "type_id": ego.type_id,
        "position_m": xyz(transform.location),
        "rpy_rad": [
            math.radians(v)
            for v in (transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw)
        ],
    }
    return fixed_wheel_points(state), profile


class ViolationSession:
    def __init__(self, directory, world, ego, corridor, identity, draw=False):
        import carla

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.world, self.ego, self.corridor = world, ego, corridor
        self.identity, self.draw = identity, draw
        profile = validate_bbox(ego)
        self.lock = threading.Lock()
        self.events, self.rows, self.errors, self.times = [], [], [], []
        self.first_seen = {}
        self.closed = False
        self.covered_from = world.get_snapshot().frame + 1
        self.sensor = world.spawn_actor(
            world.get_blueprint_library().find("sensor.other.collision"),
            carla.Transform(),
            attach_to=ego,
        )
        self.sensor.listen(self._collision)
        self.raw = (self.directory / "wheel_observations.jsonl").open("x", buffering=65536)
        self.collision_stream = (self.directory / "collision_events.jsonl").open(
            "x", buffering=65536
        )
        write_json(self.directory / "corridor.json", corridor.to_dict())
        self.config = {
            "schema_version": 2,
            **identity,
            "collision_rule": "all raw sensor events; no impulse/type/fault/spawn exclusions",
            "lane_rule": "any of four wheel XY projection points outside fixed route corridor",
            "confirmation_ticks": 1,
            "wheel_api_convention": "fixed vehicle-local profile transformed by logged ego pose",
            "wheel_profile": profile,
            "wheel_source": "vehicle_fixed_reference",
            "projection": "vertical XY ground projection; wheel hub height only selects road layer",
            "coverage_from_frame": self.covered_from,
            "pre_sensor_coverage": "unknown; not assigned safe",
            "callback_drain_wall_seconds": 0.25,
            "timing_scope": (
                "detector_wall_ms: fixed wheel transform and geometry; "
                "excludes JSON IO and collision callbacks"
            ),
            "event_stream_assumption": (
                "normal delivery within drain; event sensor has no per-frame heartbeat"
            ),
            "carla_client": None,
        }
        write_json(self.directory / "detector_config.json", self.config)

    def _collision(self, event):
        try:
            with self.lock:
                frame = int(event.frame)
                first = self.first_seen.get(event.other_actor.id)
                record = {
                    **self.identity,
                    "event_id": len(self.events),
                    "frame": frame,
                    "timestamp": float(event.timestamp),
                    "callback_wall_ns": time.time_ns(),
                    "other_actor_id": int(event.other_actor.id),
                    "other_actor_type": event.other_actor.type_id,
                    "normal_impulse": xyz(event.normal_impulse),
                    "first_observed_actor_frame": first,
                    "recently_first_observed": None if first is None else 0 <= frame - first <= 2,
                    "spawn_related_confirmed": None,
                }
                # Recently observed is metadata, NOT a reliable spawn or responsibility label.
                self.events.append(record)
                if not self.closed:
                    self.collision_stream.write(json.dumps(record, allow_nan=False) + "\n")
                    self.collision_stream.flush()
                else:
                    self.errors.append("collision callback after finalization")
        except Exception as error:
            self.errors.append(f"collision callback: {type(error).__name__}: {error}")

    def observe(self, frame, sim_time, state, actors=()):
        if self.closed:
            raise RuntimeError("Cannot observe a closed detector")
        start = time.perf_counter()
        if self.rows and frame != self.rows[-1]["frame"] + 1:
            self.errors.append(f"nonconsecutive frame: {frame}")
        with self.lock:
            for actor in actors:
                self.first_seen.setdefault(actor["id"], frame)
        try:
            points = fixed_wheel_points(state)
            outcome = classify_wheels(points, self.corridor)
        except (ValueError, RuntimeError, KeyError, TypeError) as error:
            points = []
            outcome = {
                "lane_outside_at_frame": None,
                "wheel_outside": None,
                "coverage_valid": False,
            }
            self.errors.append(f"frame {frame}: {error}")
        row = {
            **self.identity,
            "frame": int(frame),
            "sim_time_s": float(sim_time),
            "ego": state,
            "wheel_points_world_m": points,
            "wheel_source": "vehicle_fixed_reference",
            "wheel_profile_id": self.config["wheel_profile"]["id"],
            **outcome,
        }
        row["detector_wall_ms"] = (time.perf_counter() - start) * 1000
        self.times.append(row["detector_wall_ms"])
        self.rows.append(row)
        self.raw.write(json.dumps(row, allow_nan=False) + "\n")
        if len(self.rows) % 20 == 0:
            self.raw.flush()
        if self.draw:
            self.overlay(row)
        return row

    def overlay(self, row):
        import carla

        pos = row["ego"]["position_m"]
        for patch in self.corridor.patches:
            p = patch["polygon"]
            if math.dist(p[0][:2], pos[:2]) > 25:
                continue
            for a, b in zip(p, p[1:] + p[:1], strict=True):
                self.world.debug.draw_line(
                    carla.Location(a[0], a[1], a[2] + 0.12),
                    carla.Location(b[0], b[1], b[2] + 0.12),
                    thickness=0.035,
                    color=carla.Color(40, 180, 240),
                    life_time=0.12,
                )
        flags = row["wheel_outside"] or [True] * 4
        for i, (p, outside) in enumerate(zip(row["wheel_points_world_m"], flags, strict=False)):
            color = carla.Color(255, 30, 30) if outside else carla.Color(0, 255, 60)
            location = carla.Location(p[0], p[1], p[2] + 0.2)
            self.world.debug.draw_point(location, size=0.12, color=color, life_time=0.12)
            self.world.debug.draw_string(location, str(i), color=color, life_time=0.12)

    def close(self, reason="normal"):
        if self.closed:
            return
        # Do not advance physics to drain an event callback queue.
        time.sleep(0.25)
        try:
            self.sensor.stop()
        except RuntimeError as error:
            self.errors.append(f"sensor stop: {error}")
        with self.lock:
            self.closed = True
            self.raw.close()
            self.collision_stream.close()
            events = list(self.events)
        try:
            self.sensor.destroy()
        except RuntimeError as error:
            self.errors.append(f"sensor destroy: {error}")
        end = self.rows[-1]["frame"] if self.rows else self.covered_from - 1
        final = list(finalize_rows(self.rows, events, self.covered_from, end))
        with (self.directory / "violations.jsonl").open("w") as stream:
            for row in final:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
        frames = {r["frame"] for r in self.rows}
        write_json(
            self.directory / "violation_summary.json",
            {
                **self.identity,
                "closed": True,
                "close_reason": reason,
                "rows": len(final),
                "errors": self.errors,
                "collision_events": len(events),
                "events_outside_recorded_frames": [
                    e["event_id"] for e in events if e["frame"] not in frames
                ],
                "collision_episodes": collision_episodes(events),
                "violation_seen": final[-1]["violation_seen"] if final else None,
                "covered_through_frame": end,
                "detector_timing": describe(self.times),
            },
        )


def main():
    """Extend telemetry with collision and route-departure recording."""
    from leaderboard.scenarios import scenario_manager

    from ksae_2026_autumn import telemetry_runtime as base

    original_install = base.install_hooks

    def install(output):
        cleanup = original_install(output)
        cls = scenario_manager.ScenarioManager
        old_load, old_stop = cls.load_scenario, cls.stop_scenario
        sessions = []

        def load(self, scenario, agent, route_index, rep_number):
            result = old_load(self, scenario, agent, route_index, rep_number)
            ctx = self._telemetry_context
            corridor = from_route(ctx.world.get_map(), scenario.route)
            identity = {
                k: ctx.metadata[k] for k in ("run_id", "route_id", "branch_id", "repetition")
            }
            session = ViolationSession(ctx.directory, ctx.world, ctx.ego, corridor, identity)
            sessions.append(session)
            self._violation_session = session
            old_submit = ctx.submitted

            def submit(control, frame):
                row = ctx.pending
                session.observe(row["frame"], row["sim_time_s"], row["ego"], row["actors"])
                return old_submit(control, frame)

            ctx.submitted = submit
            return result

        def stop(self):
            session = getattr(self, "_violation_session", None)
            if session is not None:
                session.close("stop_scenario")
            return old_stop(self)

        cls.load_scenario, cls.stop_scenario = load, stop

        def close():
            for session in sessions:
                session.close("process_exit")
            cleanup()

        return close

    base.install_hooks = install
    base.main()


if __name__ == "__main__":
    main()
