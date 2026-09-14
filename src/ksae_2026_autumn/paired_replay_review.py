"""Audit state reconstruction and independent E/E and F/F continuations."""

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path

from .dual_control import LIMITS, Protocol, Settings, Stall, state_error
from .dual_results import lines, same
from .replay_candidate_input import fingerprint, load_payload
from .replay_checkpoint import compare as compare_checkpoint

CASES = ("E0", "E1", "E2", "F1", "F2")


def route(case):
    return Path(case) / "routes/RouteScenario_70001_rep0"


def compare_trajectories(a, b, start, stop, controls=True):
    if min(len(a), len(b)) <= stop or start < 0 or stop < start:
        raise ValueError("Incomplete trajectory comparison interval")
    maxima = {k: 0.0 for k in LIMITS}
    control_error = 0.0
    first_exceedance = None
    discrete_differences = []
    for i in range(start, stop + 1):
        match = state_error(a[i], b[i])
        for key in maxima:
            value = match["maxima"][key]
            if not math.isfinite(value):
                raise ValueError("Nonfinite state difference")
            maxima[key] = max(maxima[key], value)
        if controls:
            for key in ("throttle", "brake", "steer"):
                value = abs(a[i]["submitted_control"][key] - b[i]["submitted_control"][key])
                if not math.isfinite(value):
                    raise ValueError("Nonfinite control difference")
                control_error = max(control_error, value)
            for key in ("hand_brake", "reverse", "manual_gear_shift", "gear"):
                if a[i]["submitted_control"][key] != b[i]["submitted_control"][key]:
                    discrete_differences.append({"route_tick": i, "field": key})
        if first_exceedance is None and (any(match["maxima"][k] > LIMITS[k] for k in LIMITS)):
            first_exceedance = {
                "route_tick": i,
                "seconds_from_interval_start": (i - start) * 0.05,
            }
    return {
        "first_tolerance_exceedance": first_exceedance,
        "match": all(maxima[k] <= LIMITS[k] for k in maxima),
        "state_error_maxima": maxima,
        "state_tolerances": LIMITS,
        "command_absolute_error_max": control_error if controls else None,
        "command_tolerance": None,
        "command_error_is_gate": False,
        "discrete_command_differences": discrete_differences,
        "compared_states": stop - start + 1,
    }


def analyze_group(directory, executions):
    directory = Path(directory)
    reports = {}
    for name in CASES:
        if name not in executions:
            reports[name] = {"status": "MISSING", "errors": ["Run not performed"]}
            continue
        report = analyze_case(directory / name, "takeover" if name.startswith("F") else "delayed")
        if executions[name].get("returncode") != 0:
            report["status"] = "FAIL"
            report.setdefault("errors", []).append(str(executions[name]))
        reports[name] = report
    result = {
        "status": "BLOCKED",
        "cases": reports,
        "prefix": {},
        "checkpoints": {},
        "same_controller": {},
    }
    if any(r["status"] in ("FAIL", "MISSING") for r in reports.values()):
        return result
    try:
        rows = {n: lines(route(directory / n) / "ticks.jsonl") for n in CASES}
        violations = {n: lines(route(directory / n) / "violations.jsonl") for n in CASES}
        checkpoints = {
            n: json.loads((route(directory / n) / "checkpoint.json").read_text()) for n in CASES
        }
        candidate = checkpoints["E0"]["route_tick"]
        for name in CASES:
            cp = checkpoints[name]
            if cp["phase"] != "pre_candidate_decision" or cp["route_tick"] != candidate:
                raise ValueError("Checkpoint phase/candidate mismatch")
            if reports[name]["candidate_frame"] - rows[name][0]["frame"] != candidate:
                raise ValueError("Checkpoint frame does not match logged candidate")
            for i, row in enumerate(rows[name]):
                if row["replay"]["case"] != name or row["replay"]["route_tick"] != i:
                    raise ValueError("Incorrect case/tick record")
                expected_replay = name != "E0" and i < candidate
                if row["replay"]["prefix_controls_replayed"] != expected_replay:
                    raise ValueError("Stored future actions used or missing prefix replay")
                if row["replay"]["post_candidate_live"] != (i > candidate):
                    raise ValueError("Missing live closed-loop continuation")
                if row["replay"].get("candidate_input_shared") != (i == candidate):
                    raise ValueError("Common input used outside candidate or missing at candidate")
        metas = [json.loads((directory / n / "run.json").read_text()) for n in CASES]
        fields = (
            "seed",
            "model_sha256",
            "config_sha256",
            "routes_sha256",
            "garage_source_hashes",
            "implementation_hashes",
            "control_environment",
        )
        result["configuration_mismatches"] = [
            k
            for k in fields
            if any(k not in m for m in metas) or any(m[k] != metas[0][k] for m in metas[1:])
        ]
        simulations = [
            json.loads((route(directory / n) / "route.json").read_text())["simulation"]
            for n in CASES
        ]
        if any(s != simulations[0] for s in simulations[1:]):
            result["configuration_mismatches"].append("simulation")
        for name in CASES[1:]:
            # Candidate state is included; its newly selected control is excluded.
            state = compare_trajectories(rows["E0"], rows[name], 0, candidate, controls=False)
            control = compare_trajectories(rows["E0"], rows[name], 0, candidate - 1)
            prefix_exact = all(
                rows["E0"][i]["submitted_control"] == rows[name][i]["submitted_control"]
                for i in range(candidate)
            )
            result["prefix"][name] = {
                "stored_commands_replayed_exactly": prefix_exact,
                "match": state["match"] and prefix_exact,
                "states": state,
                "commands": control,
            }
            receipt = json.loads((route(directory / name) / "restoration.json").read_text())
            reference_sha = hashlib.sha256(
                (route(directory / "E0") / "checkpoint.json").read_bytes()
            ).hexdigest()
            if (
                receipt.get("reference_checkpoint_sha256") != reference_sha
                or receipt.get("candidate_tick") != candidate
                or receipt.get("restored_groups") != ["agent", "rng"]
                or receipt.get("post_candidate_inputs") != "live"
            ):
                raise ValueError("Invalid restoration receipt")
            before = compare_checkpoint(
                route(directory / "E0"), route(directory / name) / "before_restore"
            )
            after = compare_checkpoint(route(directory / "E0"), route(directory / name))
            if receipt.get("before") != before or receipt.get("after") != after:
                raise ValueError("Restoration audit differs from captured evidence")
            result.setdefault("restoration_audit", {})[name] = receipt
            result["checkpoints"][name] = compare_checkpoint(
                route(directory / "E0"), route(directory / name)
            )
        applications = {}
        for name in CASES:
            folder = route(directory / name)
            app = json.loads((folder / "candidate_application.json").read_text())
            payload = load_payload(folder, candidate)
            if (
                app["tick"] != candidate
                or app["forward_calls"] != 1
                or app["following_inputs"] != "live"
                or app["payload_fingerprint"] != fingerprint(payload)
                or app["backend"]["cudnn_benchmark"] is not False
                or app["backend"]["cudnn_deterministic"] is not True
                or app["backend"]["deterministic_algorithms"] is not True
            ):
                raise ValueError("Invalid candidate application evidence")
            applications[name] = app
        result["candidate_inputs"] = {
            name: {"match": applications[name] == applications["E0"]} for name in CASES
        }
        for a, b in (("E1", "E2"), ("F1", "F2"), ("E0", "E1")):
            match = compare_trajectories(rows[a], rows[b], candidate, candidate + 60)
            va = [v["violation_at_frame"] for v in violations[a][candidate : candidate + 61]]
            vb = [v["violation_at_frame"] for v in violations[b][candidate : candidate + 61]]
            match["violation_sequence_equal"] = va == vb
            match["trajectory_within_reference_limits"] = match["match"]
            match["V_H_equal"] = reports[a]["V_H"] == reports[b]["V_H"]
            match["match"] = match["V_H_equal"]
            result["same_controller"][a + "/" + b] = match
        ok = (
            all(r["status"] == "PASS" for r in reports.values())
            and not result["configuration_mismatches"]
            and all(
                r["match"]
                for group in ("prefix", "checkpoints", "candidate_inputs", "same_controller")
                for r in result[group].values()
            )
        )
        result.update(
            status="PASS" if ok else "REVIEW",
            candidate_tick=candidate,
            horizon_s=3,
            observed_V_E=reports["E1"]["V_H"],
            observed_V_F=reports["F1"]["V_H"],
        )
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        result.update(status="FAIL", error=f"{type(exc).__name__}: {exc}")
    return result


def review(root):
    root = Path(root)
    config = json.loads((root / "replay_suite.json").read_text())
    attempts = json.loads((root / "execution.json").read_text())
    after_path = root / "garage_sources_after.json"
    dependency_match = (
        not after_path.exists()
        or json.loads(after_path.read_text()) == config["garage_team_code_sha256"]
    )
    count = config["repetitions"]
    if type(count) is not int or count not in (1, 3):
        raise ValueError("Expected 1 or 3 repetitions")
    indexed = {}
    for attempt in attempts:
        key = (attempt["repetition"], attempt["case"])
        if key in indexed or key[0] not in range(count) or key[1] not in CASES:
            raise ValueError("Invalid/duplicate execution entry")
        indexed[key] = attempt
    groups = {
        str(i): analyze_group(
            root / f"rep_{i:02d}",
            {n: indexed[(i, n)] for n in CASES if (i, n) in indexed},
        )
        for i in range(count)
    }
    status = (
        "FAIL"
        if any(
            g["status"] == "FAIL" or any(c["status"] == "FAIL" for c in g["cases"].values())
            for g in groups.values()
        )
        else "PASS"
        if all(g["status"] == "PASS" for g in groups.values())
        else "REVIEW"
    )
    if not dependency_match:
        status = "FAIL"
    return {
        "schema_version": 2,
        "protocol": "common_candidate_input_live_continuation",
        "post_candidate_command_tolerance": None,
        "status": status,
        "repetitions": groups,
        "garage_dependency_check_complete": after_path.exists(),
        "scope": "Controlled Town01 common candidate input and H-label stability",
        "post_candidate_trajectory_errors_are_diagnostics": True,
        "full_snapshot_restore_validated": False,
        "risk_monitor_validated": False,
        "horizon_s": 3,
        "requires_avoidance_benefit": False,
        "recorded_attempts": len(attempts),
        "requested_attempts": count * len(CASES),
        "analysis_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }


def analyze_case(root, mode):
    errors, attention = [], []
    try:
        root = Path(root)
        dirs = list((root / "routes").glob("*"))
        if len(dirs) != 1:
            raise ValueError("Expected one route log")
        folder = dirs[0]
        rows, vv = lines(folder / "ticks.jsonl"), lines(folder / "violations.jsonl")
        summary = json.loads((folder / "summary.json").read_text())
        vs = json.loads((folder / "violation_summary.json").read_text())
        result = json.loads((root / "result.json").read_text())
        records = result["_checkpoint"]["records"]
        if len(records) != 1 or not rows:
            raise ValueError("Missing result or state rows")
        record = records[0]
        if result.get("entry_status") != "Finished" or result.get("eligible") is not True:
            errors.append("Evaluator did not finish")
        if any(w in record["status"].lower() for w in ("crash", "invalid", "started", "couldn't")):
            errors.append("Evaluator execution failure")
        for item in (summary, vs):
            if (
                not item.get("closed")
                or item.get("close_reason") != "stop_scenario"
                or item.get("errors")
            ):
                errors.append("Logging did not close normally")
        if vs.get("events_outside_recorded_frames"):
            errors.append("Collision outside state coverage")
        if [v["frame"] for v in vv] != [r["frame"] for r in rows]:
            errors.append("Violation/state coverage mismatch")
        if any(
            v.get("coverage_valid") is not True or v.get("collision_coverage_valid") is not True
            for v in vv
        ):
            errors.append("Invalid safety coverage")
        if any(not isinstance(v.get("violation_at_frame"), bool) for v in vv):
            errors.append("Missing or non-boolean safety outcome")
        if summary["rows"] != len(rows) or summary["submit_calls"] != len(rows):
            errors.append("Submission count mismatch")
        gate, channel = (
            Protocol(mode, replace(Settings(), horizon_ticks=60)),
            Stall(True),
        )
        active, events, over50, integrated = [], [], [], []
        early_exits = []
        pre_stale = 0
        for i, r in enumerate(rows):
            d = r.get("dual", r.get("restart", {}))
            if i and r["frame"] != rows[i - 1]["frame"] + 1:
                raise ValueError("Nonconsecutive frames")
            if (
                not r["frame"]
                == r["state_frame"]
                == r["decision_frame"]
                == r["command_submit_frame"]
                == d["fallback_frame"]
            ):
                errors.append("Frame mismatch")
            if any(f != r["frame"] for f in r["sensor_frames"].values()):
                errors.append("Sensor timestamp mismatch")
            onset = d["event"] == "fault_onset"
            delay = channel.step(
                r["frame"],
                r["e2e_control"],
                r["current_e2e_action_source_frame"],
                onset,
            )
            if delay != r["action_delay"]:
                errors.append("Stall replay mismatch")
            fresh = delay["actual_age_ticks"] == 0 and delay["selected_source_frame"] == r["frame"]
            diag = d["fallback_diagnostics"]
            good = (
                diag.get("emergency") is False
                and diag.get("reason") == "sampled_mpc"
                and diag.get("valid_candidates", 0) > 0
                and d["fallback_wall_ms"] <= 50
            )
            expected = gate.step(
                r["frame"],
                d["eligible"],
                d["lead_gap_m"],
                d["ego_speed_mps"],
                fresh,
                good,
                d["trigger_override"],
            )
            if (
                any(d[k] != v for k, v in expected.items())
                or d["fallback_verified"] != good
                or d["e2e_fresh"] != fresh
            ):
                errors.append("State machine replay mismatch")
            is_f = expected["selected"] == "FALLBACK"
            if "candidate_count" not in diag:
                # This pinned adapter guard exits before MPC candidate evaluation.
                # No count is fabricated; selected emergency control still fails
                # the existing `good` gate below.
                if (
                    diag.get("emergency") is True
                    and diag.get("reason") == "required_obstacle_outside_configured_range"
                ):
                    early_exits.append(
                        {
                            "frame": r["frame"],
                            "reason": diag["reason"],
                            "actually_selected": is_f,
                            "candidate_count": None,
                        }
                    )
                else:
                    errors.append(f"Unexpected missing candidate_count at frame {r['frame']}")
            elif diag["candidate_count"] != 45:
                errors.append("Fallback candidate count changed")
            selected = d["fallback_control"] if is_f else delay["output_control"]
            source = r["frame"] if is_f else delay["selected_source_frame"]
            age = 0 if is_f else delay["actual_age_ticks"]
            if (
                not same(r["submitted_control"], selected)
                or r["controller"] != expected["selected"]
            ):
                errors.append("Wrong controller/control applied")
            if r["action_source_frame"] != source or r["injected_delay_ticks"] != age:
                errors.append("Applied command age mismatch")
            if i and not same(r["previous_control_observed"], rows[i - 1]["submitted_control"]):
                errors.append("Submitted control was not observed on next frame")
            if is_f:
                active.append(r)
                if not good:
                    attention.append("Emergency/invalid/late fallback command applied")
            elif (
                gate.trigger is not None
                and r["frame"] < gate.trigger + gate.c.takeover_after_ticks
                and age >= 10
            ):
                pre_stale += 1
            if d["base_eligible"]:
                integrated.append(d["integration_wall_ms"])
                if d["fallback_wall_ms"] > 50:
                    over50.append(r["frame"])
            if d["event"]:
                events.append(
                    {
                        "event": d["event"],
                        "frame": r["frame"],
                        "speed_mps": d["ego_speed_mps"],
                        "gap_m": d["lead_gap_m"],
                    }
                )
        if gate.trigger is None:
            attention.append("FAULT_NOT_REACHED: no eligible pre-braking opportunity")
        if mode != "clean" and pre_stale == 0:
            attention.append("No >=500ms-old E2E command applied before takeover opportunity")
        candidate = None if gate.trigger is None else gate.trigger + gate.c.takeover_after_ticks
        window = (
            []
            if candidate is None
            else [v for v in vv if candidate <= v["frame"] <= candidate + gate.c.horizon_ticks]
        )
        complete_h = len(window) == gate.c.horizon_ticks + 1
        v_h = any(v["violation_at_frame"] for v in window) if complete_h else None
        if not complete_h:
            attention.append("Incomplete fixed 3s outcome horizon")
        violations = sum(bool(v["violation_at_frame"]) for v in vv)
        pre_v = candidate is not None and any(
            v["violation_at_frame"] for v in vv if v["frame"] <= candidate
        )
        if pre_v:
            attention.append("Violation already present before/at candidate")
        post = [] if gate.recovery is None else [r for r in rows if r["frame"] > gate.recovery]
        progress = (
            math.dist(post[0]["ego"]["position_m"][:2], post[-1]["ego"]["position_m"][:2])
            if post
            else 0.0
        )
        if mode == "takeover":
            if gate.takeover != candidate or gate.recovery is not None:
                attention.append("Fallback branch must start at candidate and never recover")
            if len(active) < gate.c.horizon_ticks:
                attention.append("Fallback not retained throughout outcome horizon")
        return {
            "status": "FAIL" if errors else "REVIEW" if attention else "PASS",
            "errors": sorted(set(errors)),
            "attention": sorted(set(attention)),
            "frames": len(rows),
            "events": events,
            "candidate_frame": candidate,
            "horizon_s": 3,
            "horizon_complete": complete_h,
            "V_H": v_h,
            "pre_candidate_violation": pre_v,
            "violation_frames": violations,
            "lane_departure_frames": sum(v["lane_outside_at_frame"] for v in vv)
            if all(isinstance(v.get("lane_outside_at_frame"), bool) for v in vv)
            else None,
            "collision_callbacks": vs["collision_events"],
            "fallback_frames": len(active),
            "fallback_pre_candidate_evaluation_exits": early_exits,
            "unapplied_fallback_early_exit_frames": sum(
                not x["actually_selected"] for x in early_exits
            ),
            "applied_fallback_early_exit_frames": sum(x["actually_selected"] for x in early_exits),
            "pre_takeover_stale_e2e_frames_ge_500ms": pre_stale,
            "post_recovery_frames": len(post),
            "post_recovery_progress_m": progress,
            "stop_before_lead_departure": gate.stop_seen,
            "fallback_max_ms": max(
                r.get("dual", r.get("restart", {}))["fallback_wall_ms"]
                for r in rows
                if r.get("dual", r.get("restart", {}))["base_eligible"]
            ),
            "integration_max_ms": max(integrated),
            "integration_over_50ms_frames": sum(x > 50 for x in integrated),
            "real_time_system_certified": False,
            "route_status": record["status"],
            "custom_scores": record["scores"],
            "benchmark_infractions": {k: len(v) for k, v in record["infractions"].items()},
            "benchmark_note": (
                "All evaluator infractions retained. MinSpeed is not a gate for this "
                "traffic-free custom scenario; DS is not an official benchmark result."
            ),
        }
    except (OSError, KeyError, TypeError, ValueError, IndexError) as exc:
        return {"status": "FAIL", "errors": [f"{type(exc).__name__}: {exc}"]}
