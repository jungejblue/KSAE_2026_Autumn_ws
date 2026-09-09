"""Read recorded driving data and export offline safety metric tables."""

import csv
import hashlib
import json
import math
import shutil
from collections import Counter
from pathlib import Path

from ksae_2026_autumn.route_corridor import Corridor
from ksae_2026_autumn.safety_metrics import HORIZON_S, STEP_S, collision_speeds, minimum_ttc, ttlc
from ksae_2026_autumn.violation_geometry import PROFILE, fixed_wheel_points

INPUT_FILES = (
    "ticks.jsonl",
    "violations.jsonl",
    "collision_events.jsonl",
    "corridor.json",
    "detector_config.json",
    "violation_summary.json",
)
CODE_FILES = (
    "safety_metrics.py",
    "safety_metrics_io.py",
    "route_corridor.py",
    "violation_geometry.py",
)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_hashes():
    root = Path(__file__).resolve().parent
    return {name: sha(root / name) for name in CODE_FILES}


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def read_jsonl(path):
    def reject(x):
        raise ValueError(f"Nonfinite JSON: {x}")

    return [
        json.loads(s, parse_constant=reject)
        for s in Path(path).read_text().splitlines()
        if s.strip()
    ]


def write_csv(path, rows):
    if not rows:
        Path(path).write_text("event_id,frame,collision_speed_proxy_mps,status\n")
        return
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def profile_matches(recorded):
    """Ignore only the retired provenance pathname; retain every other profile field."""
    if not isinstance(recorded, dict):
        return False
    return {k: v for k, v in recorded.items() if k != "source"} == {
        k: v for k, v in PROFILE.items() if k != "source"
    }


def compute_route(path):
    path = Path(path)
    ticks = read_jsonl(path / "ticks.jsonl")
    violations = read_jsonl(path / "violations.jsonl")
    events = read_jsonl(path / "collision_events.jsonl")
    config = json.loads((path / "detector_config.json").read_text())
    shape = json.loads((path / "corridor.json").read_text())
    summary = json.loads((path / "violation_summary.json").read_text())
    if config.get("schema_version") != 2 or not profile_matches(config.get("wheel_profile")):
        raise ValueError("Requires schema version 2 and a matching fixed wheel profile")
    if not summary.get("closed") or summary.get("errors"):
        raise ValueError("Invalid detector session")
    if not ticks or len(ticks) != len(violations):
        raise ValueError("Missing/mismatched frame records")
    if summary["collision_events"] != len(events):
        raise ValueError("Collision count mismatch")
    if [e["event_id"] for e in events] != list(range(len(events))):
        raise ValueError("Collision event IDs mismatch")
    corridor = Corridor(shape["samples"], shape["height_tolerance_m"])
    if corridor.patches != shape["patches"]:
        raise ValueError("Corridor mismatch")
    rows = []
    event_frames = {e["frame"] for e in events}
    frames = {t["frame"] for t in ticks}
    if not event_frames <= frames:
        raise ValueError("Collision outside recorded frame range")
    for i, (tick, violation) in enumerate(zip(ticks, violations, strict=True)):
        f = tick["frame"]
        if f != violation["frame"] or f != tick["state_frame"]:
            raise ValueError("Frame mismatch")
        if i and (
            f != ticks[i - 1]["frame"] + 1
            or abs(tick["sim_time_s"] - ticks[i - 1]["sim_time_s"] - STEP_S) > 1e-5
        ):
            raise ValueError("Frame/time gap")
        if tick["ego"] != violation["ego"] or not violation["coverage_valid"]:
            raise ValueError("Invalid ego/coverage")
        if violation["wheel_source"] != "vehicle_fixed_reference":
            raise ValueError("Wrong wheel source")
        points = fixed_wheel_points(tick["ego"])
        recorded = violation["wheel_points_world_m"]
        if (
            len(recorded) != 4
            or any(len(p) != 3 for p in recorded)
            or any(
                not math.isfinite(x) or abs(x - y) > 1e-7
                for a, b in zip(recorded, points, strict=True)
                for x, y in zip(a, b, strict=True)
            )
        ):
            raise ValueError("Fixed wheel transform mismatch")
        lane = not all(corridor.contains(p) for p in points)
        if (
            lane != violation["lane_outside_at_frame"]
            or (lane or f in event_frames) != violation["violation_at_frame"]
        ):
            raise ValueError("Violation mismatch")
        ttc = minimum_ttc(tick["ego"], tick["actors"], tick["unmatched_actor_ids"])
        lane_time = ttlc(tick["ego"], corridor)
        rows.append(
            {
                "route_id": tick["route_id"],
                "frame": f,
                "sim_time_s": tick["sim_time_s"],
                **ttc,
                **lane_time,
                "collision_at_frame": f in event_frames,
                "lane_outside_at_frame": lane,
            }
        )
    speeds = collision_speeds(events, ticks)
    issues = []
    if any(r["ttc_status"] == "unknown_actor_state" for r in rows):
        issues.append("Unmatched actor states: TTC unknown")
    if any(r["status"] != "previous_frame_proxy" for r in speeds):
        issues.append("Missing collision pre-frame speed")
    values = [r["ttc_s"] for r in rows if r["ttc_s"] is not None]
    lanes = [r["ttlc_s"] for r in rows if r["ttlc_s"] is not None]
    result = {
        "route_id": ticks[0]["route_id"],
        "rows": len(rows),
        "status": "REVIEW" if issues else "PASS",
        "issues": issues,
        "min_ttc_s": min(values) if values else None,
        "min_ttlc_s": min(lanes) if lanes else None,
        "ttc_status_counts": dict(Counter(r["ttc_status"] for r in rows)),
        "ttlc_status_counts": dict(Counter(r["ttlc_status"] for r in rows)),
        "collision_callbacks": len(speeds),
        "contact_episodes": sum(r["episode_start"] for r in speeds),
        "first_collision_speed_proxy_kmh": speeds[0]["collision_speed_proxy_kmh"]
        if speeds
        else None,
    }
    return rows, speeds, result


def extract(source, output):
    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    if output == source or output.is_relative_to(source):
        raise ValueError("Output must be outside source run")
    dirs = sorted(p.parent for p in (source / "routes").glob("*/ticks.jsonl"))
    if not dirs:
        raise ValueError("No recorded route ticks found")
    for p in dirs:
        for name in INPUT_FILES:
            if not (p / name).is_file():
                raise ValueError(f"Missing input: {p / name}")
    output.mkdir(parents=True, exist_ok=False)
    metadata = {
        "status": "RUNNING",
        "source": str(source),
        "code_hashes": code_hashes(),
        "input_hashes": {},
        "horizon_s": HORIZON_S,
        "ttlc_step_s": STEP_S,
        "ttc_model": (
            "constant global velocity, fixed orientation, projected 3D bbox with vertical overlap"
        ),
        "ttc_scope": "recorded vehicles and pedestrians only; static map geometry excluded",
        "ttlc_model": (
            "constant global velocity and orientation; fixed wheel profile; first sampled exit"
        ),
        "speed_definition": (
            "ego speed at event.frame-1, 0.05 s proxy before impact; not exact impact speed"
        ),
        "null_semantics": (
            "Use status column; horizon censoring/endpoint/unknown/no collision are distinct"
        ),
    }
    write_json(output / "analysis.json", metadata)
    try:
        results = []
        for src in dirs:
            dst = output / "inputs" / src.name
            dst.mkdir(parents=True)
            for name in INPUT_FILES:
                shutil.copy2(src / name, dst / name)
                metadata["input_hashes"][str((dst / name).relative_to(output))] = sha(dst / name)
            rows, speeds, summary = compute_route(dst)
            dest = output / "routes" / src.name
            dest.mkdir(parents=True)
            write_csv(dest / "metrics.csv", rows)
            write_csv(dest / "collision_speeds.csv", speeds)
            write_json(dest / "summary.json", summary)
            results.append(summary)
        write_json(output / "summary.json", results)
        metadata["status"] = "PASS" if all(x["status"] == "PASS" for x in results) else "REVIEW"
    except Exception as error:
        metadata["status"] = "FAIL"
        metadata["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        write_json(output / "analysis.json", metadata)
    return metadata
