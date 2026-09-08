"""Offline verification of telemetry and evaluator outputs; standard library only."""

import json
import math
from collections import Counter
from pathlib import Path

from ksae_2026_autumn.telemetry import describe, write_json


def check_run(directory, save=True):
    directory = Path(directory)
    errors, warnings, counts, route_reports = [], [], Counter(), []

    def fail(code, detail):
        counts[code] += 1
        if len(errors) < 100:
            errors.append({"code": code, "detail": str(detail)})

    def read(path):
        try:
            value = json.loads(path.read_text())
            if not isinstance(value, dict):
                raise ValueError("Expected a JSON object")
            return value
        except (OSError, ValueError) as error:
            fail("missing_or_invalid_json", f"{path}: {error}")
            return {}

    meta = read(directory / "run.json")
    result = read(directory / "result.json")
    try:
        if int((directory / "exit_code.txt").read_text()) != 0:
            fail("process_exit", directory)
    except (OSError, ValueError) as error:
        fail("process_exit", error)
    if meta.get("exit_code") != 0 or not meta.get("finished_at"):
        fail("run_not_finished", directory)
    if not meta.get("runtime"):
        fail("missing_runtime_metadata", directory)
    if result.get("entry_status") != "Finished" or result.get("eligible") is not True:
        fail("evaluator_not_finished", directory)
    checkpoint = result.get("_checkpoint", {})
    records = checkpoint.get("records", [])
    expected = meta.get("expected_route_ids", [])
    actual = [r.get("route_id") for r in records]
    if not expected or len(actual) != len(set(actual)) or set(actual) != set(expected):
        fail("route_set", {"expected": expected, "actual": actual})
    if checkpoint.get("progress") != [len(expected), len(expected)]:
        fail("progress", checkpoint.get("progress"))
    route_dirs = sorted((directory / "routes").glob("*/route.json"))
    if {p.parent.name for p in route_dirs} != set(expected):
        fail("route_files", [p.parent.name for p in route_dirs])

    for record in records:
        status = str(record.get("status", ""))
        if (
            any(s in status.lower() for s in ("crash", "couldn't", "invalid", "started"))
            or not status
        ):
            fail("agent_or_simulator_failure", record)
        elif status.startswith("Failed"):
            warnings.append({"route_id": record["route_id"], "driving_outcome": status})
        if "scores" not in record or "infractions" not in record:
            fail("incomplete_result", record.get("route_id"))

    for path in route_dirs:
        route = read(path)
        summary = read(path.parent / "summary.json")
        rid = route.get("route_id", path.parent.name)
        if route.get("run_id") != meta.get("run_id") or rid != path.parent.name:
            fail("route_identity", rid)
        if route.get("logging_enabled") != meta.get("logging_enabled"):
            fail("logging_mode", rid)
        if summary.get("logging_enabled") != meta.get("logging_enabled"):
            fail("summary_logging_mode", rid)
        if not summary.get("closed") or summary.get("close_reason") != "stop_scenario":
            fail("route_not_closed", rid)
        if summary.get("errors"):
            fail("logger_exception", {"route": rid, "errors": summary["errors"]})
        call_counts = [summary.get(k, -1) for k in ("agent_calls", "agent_returns", "submit_calls")]
        if len(set(call_counts)) != 1 or call_counts[0] <= 0:
            fail("call_count", {"route": rid, "counts": call_counts})
        simulation = route.get("simulation", {})
        if (
            not simulation.get("synchronous_mode")
            or abs(simulation.get("fixed_delta_seconds", 0) - 0.05) > 1e-5
        ):
            fail("simulation_settings", rid)
        report = {
            "route_id": rid,
            "agent_calls": call_counts[0],
            "rows_read": 0,
            "agent_step_active": summary.get("agent_step_active"),
            "logger_capture_and_write": summary.get("logger_capture_and_write"),
        }
        if not meta.get("logging_enabled"):
            if summary.get("rows") != 0 or (path.parent / "ticks.jsonl").exists():
                fail("unexpected_off_logging", rid)
            route_reports.append(report)
            continue
        previous, first, seen, times = None, None, set(), []
        required_sensors = {
            s["id"] for s in route.get("sensor_specs", []) if s["type"] != "sensor.opendrive_map"
        }
        if not required_sensors:
            fail("missing_sensor_specs", rid)
        unmatched = 0
        observed_control_differences = 0
        try:
            with (path.parent / "ticks.jsonl").open(encoding="utf-8") as stream:
                for line_number, line in enumerate(stream, 1):
                    location = f"{rid}:{line_number}"
                    try:
                        row = json.loads(line)
                        report["rows_read"] += 1
                        if not line.endswith("\n"):
                            fail("truncated_line", location)
                        frame = row["frame"]
                        key = (
                            row["run_id"],
                            row["route_id"],
                            row["repetition"],
                            row["branch_id"],
                            frame,
                        )
                        if key in seen:
                            fail("duplicate_frame", location)
                        seen.add(key)
                        if any(
                            row[k] != route[k]
                            for k in ("run_id", "route_id", "repetition", "branch_id")
                        ):
                            fail("mixed_identity", location)
                        if row["schema_version"] != 1:
                            fail("schema_version", location)
                        if any(
                            row[k] != frame
                            for k in ("state_frame", "decision_frame", "command_submit_frame")
                        ):
                            fail("phase_frame", location)
                        if previous:
                            if frame != previous["frame"] + 1:
                                fail("frame_gap_or_reverse", location)
                            if abs(row["sim_time_s"] - previous["sim_time_s"] - 0.05) > 1e-5:
                                fail("time_step", location)
                            if abs(row["agent_time_s"] - previous["agent_time_s"] - 0.05) > 1e-5:
                                fail("agent_time_step", location)
                            if any(
                                abs(
                                    row["previous_control_observed"][name]
                                    - previous["submitted_control"][name]
                                )
                                > 1e-6
                                for name in ("throttle", "brake", "steer")
                            ):
                                observed_control_differences += 1
                        frames = row["sensor_frames"]
                        if not required_sensors.issubset(frames):
                            fail("missing_sensor", location)
                        # The pinned Garage SensorInterface explicitly selects the current frame.
                        if any(frames[s] != frame for s in required_sensors if s in frames):
                            fail("sensor_frame", location)
                        if row["action_source_frame"] != max(frames.values()):
                            fail("action_source", location)
                        if (
                            abs(
                                row["action_observation_age_s"]
                                - (frame - row["action_source_frame"]) * 0.05
                            )
                            > 1e-8
                        ):
                            fail("action_age", location)
                        if row["controller"] != "E2E" or row["injected_delay_ticks"] != 0:
                            fail("unexpected_policy", location)
                        if row["warmup"] == row["inference_ran"]:
                            fail("warmup_flag", location)
                        for stage in ("e2e_control", "selected_control", "submitted_control"):
                            control = row[stage]
                            for name, lower, upper in (
                                ("throttle", 0, 1),
                                ("brake", 0, 1),
                                ("steer", -1, 1),
                            ):
                                value = control[name]
                                if not math.isfinite(value) or not lower <= value <= upper:
                                    fail("control_range", f"{location}:{stage}:{name}")
                        for name in ("throttle", "brake", "steer", "hand_brake", "reverse", "gear"):
                            if row["e2e_control"][name] != row["submitted_control"][name]:
                                fail("changed_control", f"{location}:{name}")
                        if row["selected_control"] != row["submitted_control"]:
                            fail("selected_vs_submitted", location)
                        if row["submitted_control"]["manual_gear_shift"] is not False:
                            fail("gear_policy", location)
                        for actor in [row["ego"], *row["actors"]]:
                            for name in (
                                "position_m",
                                "rpy_rad",
                                "velocity_mps",
                                "acceleration_mps2",
                                "angular_velocity_radps",
                                "bbox_center_local_m",
                                "bbox_extent_m",
                                "bbox_rpy_local_rad",
                            ):
                                values = actor[name]
                                if len(values) != 3 or not all(math.isfinite(v) for v in values):
                                    fail("state_numbers", f"{location}:{name}")
                            if not math.isfinite(actor["speed_mps"]) or actor["speed_mps"] < 0:
                                fail("speed_number", location)
                        for name in (
                            "sim_time_s",
                            "agent_time_s",
                            "agent_step_wall_ms",
                            "capture_wall_ms",
                            "action_observation_age_s",
                        ):
                            if not math.isfinite(row[name]) or row[name] < 0:
                                fail("time_number", f"{location}:{name}")
                        unmatched += len(row["unmatched_actor_ids"])
                        if row["inference_ran"]:
                            times.append(row["agent_step_wall_ms"])
                        if first is None:
                            first = row
                        previous = row
                    except (ValueError, KeyError, TypeError, OverflowError) as error:
                        fail("invalid_row", f"{location}: {error}")
        except OSError as error:
            fail("missing_ticks", error)
        if report["rows_read"] != summary.get("rows") or report["rows_read"] != call_counts[0]:
            fail(
                "row_count",
                {
                    "route": rid,
                    "read": report["rows_read"],
                    "summary": summary.get("rows"),
                    "calls": call_counts[0],
                },
            )
        if first is None or previous is None:
            fail("empty_ticks", rid)
        else:
            if not first["warmup"] or first["warmup_reason"] != "initialization":
                fail("missing_initialization", rid)
            if first["frame"] != summary.get("first_frame") or previous["frame"] != summary.get(
                "last_frame"
            ):
                fail("boundary_frames", rid)
        if not times:
            fail("no_active_inference", rid)
        if unmatched:
            warnings.append({"route_id": rid, "actor_snapshot_metadata_races": unmatched})
        if observed_control_differences:
            warnings.append(
                {
                    "route_id": rid,
                    "subsequently_observed_control_differences": observed_control_differences,
                }
            )
        report["active_times_recomputed"] = describe(times)
        route_reports.append(report)
    report = {
        "status": "FAIL" if counts else ("REVIEW" if warnings else "PASS"),
        "run_directory": str(directory),
        "error_counts": dict(counts),
        "errors": errors,
        "warnings": warnings,
        "routes": route_reports,
        "meaning": (
            "Structural telemetry check; not proof of safety or exact trajectory reproducibility."
        ),
    }
    if save and directory.is_dir():
        write_json(directory / "telemetry_check.json", report)
    return report
