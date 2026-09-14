"""Audit route-start reconstruction at a pre-decision boundary; never load pickle."""

import enum
import hashlib
import json
import math
import random
from collections import deque
from pathlib import Path

import numpy as np


class Encoder:
    def __init__(self):
        self.arrays = {}

    def encode(self, obj):
        if isinstance(obj, enum.Enum):
            return {"enum": type(obj).__name__, "value": self.encode(obj.value)}
        if obj is None or isinstance(obj, (str, bool, int)):
            return obj
        if isinstance(obj, (float, np.floating)):
            if not math.isfinite(obj):
                raise ValueError("Nonfinite checkpoint scalar")
            return float(obj)
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.bool_):
            return bool(obj)
        if hasattr(obj, "detach"):
            obj = obj.detach().cpu().numpy()
        if isinstance(obj, np.ndarray):
            if obj.dtype.hasobject or not np.isfinite(obj).all():
                raise ValueError("Invalid checkpoint array")
            key = f"array_{len(self.arrays):04d}"
            self.arrays[key] = obj.copy()
            return {"array": key, "shape": list(obj.shape), "dtype": str(obj.dtype)}
        if isinstance(obj, deque):
            return {"deque": [self.encode(x) for x in obj], "maxlen": obj.maxlen}
        if isinstance(obj, (tuple, list)):
            return [self.encode(x) for x in obj]
        if isinstance(obj, dict):
            if any(not isinstance(k, str) for k in obj):
                raise ValueError("Checkpoint keys must be strings")
            return {k: self.encode(obj[k]) for k in sorted(obj)}
        if all(hasattr(obj, k) for k in ("throttle", "brake", "steer")):
            return {
                k: getattr(obj, k)
                for k in (
                    "throttle",
                    "brake",
                    "steer",
                    "hand_brake",
                    "reverse",
                    "manual_gear_shift",
                    "gear",
                )
            }
        raise TypeError(f"Uncaptured checkpoint type: {type(obj).__module__}.{type(obj).__name__}")


def attributes(obj, required, optional=()):
    missing = [k for k in required if not hasattr(obj, k)]
    if missing:
        raise ValueError(f"Missing state in {type(obj).__name__}: {missing}")
    return {k: getattr(obj, k) for k in (*required, *optional) if hasattr(obj, k)}


def agent_state(agent):
    values = attributes(
        agent,
        (
            "step",
            "initialized",
            "filter_initialized",
            "stuck_detector",
            "force_move",
            "commands",
            "target_point_prev",
            "state_log",
            "lidar_buffer",
            "lidar_last",
            "control",
            "bb_buffer",
            "stop_sign_buffer",
            "clear_stop_sign",
            "lat_ref",
            "lon_ref",
        ),
        ("pred_wp", "meters_travelled"),
    )
    values["ukf"] = attributes(
        agent.ukf,
        ("x", "P", "Q", "R"),
        (
            "sigmas_f",
            "sigmas_h",
            "x_prior",
            "P_prior",
            "x_post",
            "P_post",
            "y",
            "K",
            "S",
            "SI",
            "Wm",
            "Wc",
        ),
    )
    values["route_planner"] = attributes(
        agent._route_planner,
        (
            "route",
            "route_distances",
            "saved_route",
            "saved_route_distances",
            "is_last",
            "lat_ref",
            "lon_ref",
            "min_distance",
            "max_distance",
        ),
    )
    values["network_controllers"] = []
    if len(agent.nets) != 1:
        raise ValueError("Expected the pinned single-model TF++")
    for net in agent.nets:
        state = {}
        for name in (
            "lateral_pid_controller",
            "turn_controller",
            "speed_controller",
            "turn_controller_direct",
            "speed_controller_direct",
        ):
            controller = getattr(net, name)
            # These controller classes contain only scalar settings and window state.
            state[name] = dict(vars(controller))
        values["network_controllers"].append(state)
    return values


def fallback_state(fallback, frame, seconds):
    fb = fallback

    def sign_key(identifier):
        actor = fb.world.get_actor(identifier)
        if actor is None:
            raise ValueError("Stop-sign identity no longer resolves")
        p = actor.get_location()
        return f"{actor.type_id}:{p.x:.6f},{p.y:.6f},{p.z:.6f}"

    return {
        "route_progress": fb.route._progress,
        "controller": attributes(fb.controller, ("_recovery_initial", "_recovery_ticks")),
        "planner": attributes(fb.controller.planner, ("last_offset", "recovery")),
        "mpc": attributes(
            fb.controller.mpc,
            (
                "applied_steer",
                "previous_acceleration",
                "coast_active",
                "policies",
            ),
        ),
        "actuator": {k: v for k, v in vars(fb.actuator).items() if k != "c"},
        "adapter": {
            "last_frame_offset": None if fb.last_frame is None else fb.last_frame - frame,
            "last_time_offset": None if fb.last_time is None else fb.last_time - seconds,
            "invalid_episode": fb._invalid_episode,
            "last_result": fb.last_result,
            "served_signs": sorted(sign_key(i) for i in fb.served_signs),
            "sign_wait": {sign_key(k): v - seconds for k, v in fb.sign_wait.items()},
        },
    }


def channel_state(channel, frame):
    def command(value):
        if value is None:
            return None
        control, generation, source = value
        return [control, generation - frame, None if source is None else source - frame]

    return {
        "enabled": channel.enabled,
        "duration": channel.duration,
        "onset_offset": None if channel.onset is None else channel.onset - frame,
        "last_frame_offset": None if channel.last_frame is None else channel.last_frame - frame,
        "previous": command(channel.previous),
        "held": command(channel.held),
    }


def capture(folder, agent, fallback, channel, snapshot, first_frame, onset_tick, inputs):
    """Capture before candidate TF++ call and before choosing E or F."""
    import torch

    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    frame = int(snapshot.frame)
    seconds = float(snapshot.timestamp.elapsed_seconds)
    lights = []
    for light in fallback.traffic_lights:
        p = light.get_location()
        lights.append(
            {
                "position": [p.x, p.y, p.z],
                "state": str(light.get_state()),
                "frozen": light.is_frozen(),
            }
        )
    lights.sort(key=lambda x: x["position"])
    values = {
        "agent": agent_state(agent),
        "fallback": fallback_state(fallback, frame, seconds),
        "channel": channel_state(channel, frame),
        "fixture": {
            "onset_tick": onset_tick,
            "candidate_tick": frame - first_frame,
            "lead_released": False,
            "traffic_lights": lights,
        },
        "rng": {
            "python": random.getstate(),
            "numpy": np.random.get_state(),
            "torch_cpu": torch.get_rng_state(),
            "torch_cuda": torch.cuda.get_rng_state_all(),
        },
    }
    encoder = Encoder()
    encoded = encoder.encode(values)
    array_file = folder / "checkpoint_arrays.npz"
    np.savez_compressed(array_file, **encoder.arrays)
    sensors = {}
    for name, (sensor_frame, data) in sorted(inputs.items()):
        e = Encoder()
        structure = e.encode(data)
        digest = hashlib.sha256(json.dumps(structure, sort_keys=True).encode())
        for value in e.arrays.values():
            digest.update(value.tobytes())
        sensors[name] = {
            "frame_offset": int(sensor_frame) - frame,
            "sha256": digest.hexdigest(),
        }
    # Raw candidate RGB permits later pixel-level diagnosis; it is never fed to the agent.
    if "rgb_front" in inputs:
        np.save(
            folder / "candidate_rgb_front.npy",
            inputs["rgb_front"][1],
            allow_pickle=False,
        )
    result = {
        "schema_version": 1,
        "phase": "pre_candidate_decision",
        "method": "route_start_control_prefix_reconstruction",
        "frame": frame,
        "route_tick": frame - first_frame,
        "snapshot_elapsed_seconds": seconds,
        "state": encoded,
        "sensor_packet_hashes": sensors,
        "arrays_sha256": hashlib.sha256(array_file.read_bytes()).hexdigest(),
        "full_simulator_snapshot": False,
        "coverage": [
            "TF++ UKF, LiDAR history, route planner, PID windows and counters",
            "Python/NumPy/Torch RNG states",
            "stall held/previous command and timing",
            "fallback planner/MPC/actuator/adapter state",
            "frozen fixture lights",
        ],
        "not_serialized": [
            "Unreal physics internals",
            "sensor server queues",
            "generic ScenarioRunner behavior tree",
            "Traffic Manager internals",
        ],
        "array_loading": "numpy.load(..., allow_pickle=False)",
    }
    (folder / "checkpoint.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def compare(folder_a, folder_b, atol=1e-6):
    """Discrete state exact, finite floating state with fixed absolute tolerance."""
    a, b = [json.loads((Path(f) / "checkpoint.json").read_text()) for f in (folder_a, folder_b)]
    for doc in (a, b):
        if (
            doc.get("schema_version") != 1
            or doc.get("phase") != "pre_candidate_decision"
            or set(doc.get("state", {})) != {"agent", "fallback", "channel", "fixture", "rng"}
            or not doc.get("sensor_packet_hashes")
        ):
            raise ValueError("Incomplete checkpoint schema")
    arrays = []
    for f, doc in zip((folder_a, folder_b), (a, b), strict=True):
        path = Path(f) / "checkpoint_arrays.npz"
        if hashlib.sha256(path.read_bytes()).hexdigest() != doc["arrays_sha256"]:
            raise ValueError("Checkpoint array checksum mismatch")
        arrays.append(np.load(path, allow_pickle=False))
    mismatches = []
    differences = {}

    def walk(x, y, path):
        if isinstance(x, dict) and "array" in x:
            if not isinstance(y, dict) or "array" not in y:
                mismatches.append(path)
                return
            aa, bb = arrays[0][x["array"]], arrays[1][y["array"]]
            equal = aa.shape == bb.shape and aa.dtype == bb.dtype
            if equal:
                equal = (
                    np.isfinite(aa).all()
                    and np.isfinite(bb).all()
                    and (
                        np.allclose(aa, bb, rtol=0, atol=atol)
                        if aa.dtype.kind in "fc"
                        else np.array_equal(aa, bb)
                    )
                )
            if not equal:
                mismatches.append(path)
                if aa.shape == bb.shape and aa.dtype.kind in "fiu" and bb.dtype.kind in "fiu":
                    differences[path] = float(
                        np.max(
                            np.abs(aa.astype(np.float64) - bb.astype(np.float64)),
                            initial=0,
                        )
                    )
        elif isinstance(x, dict) and isinstance(y, dict) and x.keys() == y.keys():
            for key in x:
                walk(x[key], y[key], path + "." + key)
        elif isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
            for i, (xx, yy) in enumerate(zip(x, y, strict=True)):
                walk(xx, yy, f"{path}[{i}]")
        elif isinstance(x, float) and isinstance(y, (int, float)) and not isinstance(y, bool):
            if not math.isfinite(x) or not math.isfinite(y) or abs(x - y) > atol:
                mismatches.append(path)
                differences[path] = abs(x - y)
        elif type(x) is not type(y) or x != y:
            mismatches.append(path)

    try:
        walk(a["state"], b["state"], "state")
    finally:
        for array in arrays:
            array.close()
    sensor_equal = a["sensor_packet_hashes"] == b["sensor_packet_hashes"]
    return {
        "match": not mismatches,
        "mismatched_fields": mismatches,
        "numeric_max_absolute_differences": differences,
        "floating_absolute_tolerance": atol,
        "sensor_packet_hashes_equal": sensor_equal,
        "sensor_hash_note": "Diagnostic only; require closed-loop repeatability separately.",
        "full_simulator_snapshot": False,
    }


class Decoder:
    """Decode only captured data; recover runtime types from initialized objects."""

    def __init__(self, arrays):
        self.arrays = arrays

    def decode(self, value, template=None):
        if isinstance(value, dict) and "array" in value:
            array = self.arrays[value["array"]].copy()
            if (
                list(array.shape) != value["shape"]
                or str(array.dtype) != value["dtype"]
                or array.dtype.hasobject
                or not np.isfinite(array).all()
            ):
                raise ValueError("Invalid encoded array metadata")
            if hasattr(template, "detach"):
                import torch

                return torch.as_tensor(array, device=template.device, dtype=template.dtype)
            return array
        if isinstance(value, dict) and "enum" in value:
            if not isinstance(template, enum.Enum) or type(template).__name__ != value["enum"]:
                raise ValueError("Cannot resolve enum from initialized state")
            return type(template)(value["value"])
        if isinstance(value, dict) and "deque" in value:
            old = list(template) if template is not None else []
            return deque(self.sequence(value["deque"], old), maxlen=value["maxlen"])
        if isinstance(value, list):
            old = template if isinstance(template, (list, tuple)) else []
            result = self.sequence(value, old)
            return tuple(result) if isinstance(template, tuple) else result
        if isinstance(value, dict):
            if template is not None and all(
                hasattr(template, k) for k in ("throttle", "brake", "steer")
            ):
                control = type(template)()
                if set(value) != {
                    "throttle",
                    "brake",
                    "steer",
                    "hand_brake",
                    "reverse",
                    "manual_gear_shift",
                    "gear",
                }:
                    raise ValueError("Unexpected control fields")
                for key, item in value.items():
                    setattr(control, key, item)
                return control
            old = template if isinstance(template, dict) else {}
            return {k: self.decode(v, old.get(k)) for k, v in value.items()}
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Nonfinite decoded scalar")
        return value

    def sequence(self, values, old):
        return [
            self.decode(v, old[i] if i < len(old) else old[0] if len(old) else None)
            for i, v in enumerate(values)
        ]


def restore_agent_rng(folder, agent, candidate_tick):
    """Restore software state only, before the candidate's live sensor processing."""
    import torch

    folder = Path(folder)
    document = json.loads((folder / "checkpoint.json").read_text())
    if (
        document.get("phase") != "pre_candidate_decision"
        or document.get("route_tick") != candidate_tick
    ):
        raise ValueError("Restoration boundary mismatch")
    # Validates the complete schema and archive checksum before any mutation.
    if not compare(folder, folder, atol=0)["match"]:
        raise ValueError("Invalid reference checkpoint")
    template = agent_state(agent)
    with np.load(folder / "checkpoint_arrays.npz", allow_pickle=False) as arrays:
        decoder = Decoder(arrays)
        saved = document["state"]["agent"]
        if saved.keys() != template.keys():
            raise ValueError("Agent fields differ from reference")
        for group in ("ukf", "route_planner"):
            if saved[group].keys() != template[group].keys():
                raise ValueError("Nested agent fields differ: " + group)
        if len(saved["network_controllers"]) != len(agent.nets):
            raise ValueError("Controller count differs")
        for reference_net, current_net in zip(
            saved["network_controllers"], template["network_controllers"], strict=True
        ):
            if reference_net.keys() != current_net.keys() or any(
                reference_net[k].keys() != current_net[k].keys() for k in current_net
            ):
                raise ValueError("Controller fields differ")
        decoded = decoder.decode(saved, template)
        rng = decoder.decode(
            document["state"]["rng"],
            {
                "python": random.getstate(),
                "numpy": np.random.get_state(),
                "torch_cpu": torch.get_rng_state(),
                "torch_cuda": torch.cuda.get_rng_state_all(),
            },
        )
    if len(rng["torch_cuda"]) != torch.cuda.device_count():
        raise ValueError("CUDA RNG device count differs")
    for key, value in decoded.items():
        if key not in ("ukf", "route_planner", "network_controllers"):
            setattr(agent, key, value)
    for group, target in (("ukf", agent.ukf), ("route_planner", agent._route_planner)):
        for key, value in decoded[group].items():
            setattr(target, key, value)
    for net, values in zip(agent.nets, decoded["network_controllers"], strict=True):
        for name, fields in values.items():
            for key, value in fields.items():
                setattr(getattr(net, name), key, value)
    random.setstate(rng["python"])
    np.random.set_state(rng["numpy"])
    torch.set_rng_state(rng["torch_cpu"].cpu())
    torch.cuda.set_rng_state_all(rng["torch_cuda"])
    return hashlib.sha256((folder / "checkpoint.json").read_bytes()).hexdigest()
