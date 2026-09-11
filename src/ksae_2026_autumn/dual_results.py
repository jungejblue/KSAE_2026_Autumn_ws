"""Compute handover events, safety outcomes, and matched-prefix comparisons."""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

from .dual_control import LIMITS, Protocol, Stall, state_error


def lines(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]


def same(a, b):
    return all(
        math.isfinite(float(a[k]))
        and math.isfinite(float(b[k]))
        and abs(float(a[k]) - float(b[k])) <= 1e-6
        for k in ("throttle", "brake", "steer")
    )


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
        if mode != "delayed" and record["status"] != "Completed":
            attention.append("Clean/takeover route did not complete")
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
        gate, channel = Protocol(mode), Stall(mode != "clean")
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
            attention.append("Incomplete fixed 5s outcome horizon")
        violations = sum(bool(v["violation_at_frame"]) for v in vv)
        pre_v = candidate is not None and any(
            v["violation_at_frame"] for v in vv if v["frame"] <= candidate
        )
        if pre_v:
            attention.append("Violation already present before/at candidate")
        if mode != "delayed" and (violations or vs["collision_events"]):
            attention.append("Collision/departure in clean or takeover")
        post = [] if gate.recovery is None else [r for r in rows if r["frame"] > gate.recovery]
        progress = (
            math.dist(post[0]["ego"]["position_m"][:2], post[-1]["ego"]["position_m"][:2])
            if post
            else 0.0
        )
        if mode == "takeover":
            if gate.takeover is None or gate.recovery is None:
                attention.append("Takeover/recovery missing")
            if not gate.stop_seen:
                attention.append("No stop before lead departure")
            if len(post) < gate.c.post_recovery_ticks or progress < 5:
                attention.append("Insufficient post-recovery observation/progress")
            if len(active) < gate.c.horizon_ticks:
                attention.append("Fallback not retained throughout outcome horizon")
        if over50:
            attention.append("Fallback computation exceeded 50ms")
        return {
            "status": "FAIL" if errors else "REVIEW" if attention else "PASS",
            "errors": sorted(set(errors)),
            "attention": sorted(set(attention)),
            "frames": len(rows),
            "events": events,
            "candidate_frame": candidate,
            "horizon_s": 5,
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


def analyze_pair(root, delayed_report, takeover_report):
    required = {"status", "candidate_frame", "V_H", "pre_candidate_violation"}
    problems = {}
    for mode, report in (("delayed", delayed_report), ("takeover", takeover_report)):
        missing = sorted(required - report.keys())
        if missing or report.get("status") == "FAIL":
            problems[mode] = {
                "missing_fields": missing,
                "status": report.get("status"),
                "errors": report.get("errors", []),
            }
    if problems:
        return {
            "status": "BLOCKED",
            "reason_code": "UPSTREAM_CASE_REPORT_UNAVAILABLE",
            "reason": (
                "Comparison requires complete case reports; "
                "resolve the listed upstream errors first."
            ),
            "dependencies": problems,
            "stress_benefit_demonstrated": False,
        }
    try:
        root = Path(root)

        def read(mode, name):
            return lines(root / mode / "routes/RouteScenario_70001_rep0" / name)

        e, f = read("delayed", "ticks.jsonl"), read("takeover", "ticks.jsonl")
        ce, cf = delayed_report["candidate_frame"], takeover_report["candidate_frame"]
        if ce is None or cf is None:
            raise ValueError("Candidate not reached")
        index = ce - e[0]["frame"]
        if cf - f[0]["frame"] != index or len(f) <= index:
            raise ValueError("Candidate relative frame differs")
        maxima = {k: 0.0 for k in LIMITS}
        for i in range(index + 1):
            match = state_error(f[i], e[i])
            for k in maxima:
                maxima[k] = max(maxima[k], match["maxima"][k])
            if i < index and not same(f[i]["submitted_control"], e[i]["submitted_control"]):
                raise ValueError("Prefix commands differ")
        matched = all(maxima[k] <= LIMITS[k] for k in LIMITS)
        demonstrated = (
            matched
            and delayed_report["V_H"] is True
            and takeover_report["V_H"] is False
            and not delayed_report["pre_candidate_violation"]
            and not takeover_report["pre_candidate_violation"]
            and delayed_report["status"] == takeover_report["status"] == "PASS"
        )
        return {
            "status": "PASS" if demonstrated else "REVIEW",
            "state_match": matched,
            "prefix_error_maxima": maxima,
            "tolerances": LIMITS,
            "V_E_H": delayed_report["V_H"],
            "V_F_H": takeover_report["V_H"],
            "stress_benefit_demonstrated": demonstrated,
            "reason": "Observed matched-prefix stress avoidance demonstrated"
            if demonstrated
            else (
                "Check state match, pre-existing violations, outcome horizon, and case reports; "
                "safe E2E is not a failure of the Fallback."
            ),
            "full_checkpoint_restore": False,
            "scope": (
                "fixed lead, replayed ego controls, observed motion-state matching; "
                "no controller/RNG/physics-internal snapshot claim"
            ),
        }
    except (OSError, KeyError, TypeError, ValueError, IndexError) as exc:
        return {"status": "FAIL", "error": f"{type(exc).__name__}: {exc}"}


def summarize_runs(root):
    """Return every attempt and separate completed execution from demonstrated benefit."""
    root = Path(root).expanduser().resolve()
    execution_file, config_file = root / "execution.json", root / "suite_config.json"
    executions = json.loads(execution_file.read_text())
    config = json.loads(config_file.read_text())
    count = config["repetitions"]
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 3:
        raise ValueError("Expected 1 to 3 requested repetitions")
    index = {}
    for attempt in executions:
        key = (attempt["repetition"], attempt["mode"])
        if (
            key in index
            or key[0] not in range(count)
            or key[1] not in ("clean", "delayed", "takeover")
        ):
            raise ValueError("Duplicate or invalid execution record")
        index[key] = attempt
    repetitions, table, source_files = {}, [], [execution_file, config_file]
    for rep in range(count):
        directory = root / f"rep_{rep:02d}"
        reports = {}
        for mode in ("clean", "delayed", "takeover"):
            attempt = index.get((rep, mode))
            if attempt is None:
                reports[mode] = {"status": "MISSING", "errors": ["Run not performed"]}
                continue
            case = directory / mode
            report = analyze_case(case, mode)
            if attempt.get("returncode") != 0:
                report["status"] = "FAIL"
                report.setdefault("errors", []).append(
                    "Execution error: " + str(attempt.get("error", attempt.get("returncode")))
                )
            reports[mode] = report
            for path in [case / "result.json", case / "run.json"]:
                if path.is_file():
                    source_files.append(path)
            for route in (case / "routes").glob("*"):
                source_files.extend(p for p in route.glob("*") if p.is_file())
            events = {e["event"]: e["frame"] for e in report.get("events", [])}
            takeover, recovery = events.get("takeover_forced"), events.get("recovery")
            table.append(
                {
                    "repetition": rep,
                    "mode": mode,
                    "case_status": report["status"],
                    "route_status": report.get("route_status"),
                    "V_H": report.get("V_H"),
                    "violation_frames": report.get("violation_frames"),
                    "lane_departure_frames": report.get("lane_departure_frames"),
                    "collision_callbacks": report.get("collision_callbacks"),
                    "takeover_frame": takeover,
                    "recovery_frame": recovery,
                    "fallback_duration_s": (recovery - takeover) * 0.05
                    if recovery is not None and takeover is not None
                    else None,
                    "post_recovery_frames": report.get("post_recovery_frames"),
                    "post_recovery_progress_m": report.get("post_recovery_progress_m"),
                    "fallback_max_ms": report.get("fallback_max_ms"),
                    "integration_max_ms": report.get("integration_max_ms"),
                    "integration_over_50ms_frames": report.get("integration_over_50ms_frames"),
                }
            )
        pair = analyze_pair(directory, reports["delayed"], reports["takeover"])
        # A motion-state match alone cannot establish equal models/configuration.
        fields = (
            "seed",
            "model_sha256",
            "config_sha256",
            "routes_sha256",
            "garage_source_hashes",
            "implementation_hashes",
            "control_environment",
        )
        try:
            metas = [
                json.loads((directory / m / "run.json").read_text())
                for m in ("clean", "delayed", "takeover")
            ]
            mismatches = [
                k
                for k in fields
                if any(k not in m for m in metas) or any(m[k] != metas[0][k] for m in metas[1:])
            ]
        except (OSError, ValueError, TypeError):
            mismatches = ["run_metadata_unavailable"]
        pair["configuration_match"] = not mismatches
        pair["configuration_mismatches"] = mismatches
        clean_valid = reports["clean"].get("status") == "PASS"
        if mismatches or not clean_valid:
            pair["stress_benefit_demonstrated"] = False
            if pair["status"] == "PASS":
                pair["status"] = "REVIEW"
            pair["reason"] = "Require matching run configurations and a valid clean baseline."
        repetitions[str(rep)] = {"cases": reports, "comparison": pair}
    cases = [c for rep in repetitions.values() for c in rep["cases"].values()]
    status = (
        "ERROR"
        if any(c["status"] == "FAIL" for c in cases)
        else "INCOMPLETE"
        if any(c["status"] == "MISSING" for c in cases)
        else "COMPLETE"
    )
    return {
        "schema_version": 1,
        "status": status,
        "requested_repetitions": count,
        "recorded_runs": len(executions),
        "benefit_supported_pairs": sum(
            r["comparison"].get("stress_benefit_demonstrated") is True for r in repetitions.values()
        ),
        "repetitions": repetitions,
        "metrics": table,
        "windowed": bool(config.get("windowed", False)),
        "input_hashes": {
            str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(set(source_files))
        },
        "analysis_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "interpretation": {
            "COMPLETE": "All requested runs produced analyzable records; not a safety claim.",
            "PASS": "The case or comparison meets the documented scenario conditions.",
            "REVIEW": "Conditions do not support this comparison; inspect attention/reason.",
            "FAIL": "Execution error, malformed records, or command/state inconsistency.",
            "BLOCKED": "Required case data are unavailable.",
        },
        "limitations": [
            "Forced switch after a three-second hold-last-command fault; not a risk monitor.",
            "Observed prefix matching is not complete simulator/controller/RNG restoration.",
            "Safety means collision OR fixed-wheel route departure in a five-second window.",
            "Safe delayed E2E does not demonstrate a benefit, and is not a fallback failure.",
            "Custom-route scores are not official Bench2Drive results.",
            "TF++ and fallback run sequentially; report total time separately from fallback time.",
        ],
    }


def write_comparison(output, report):
    output = Path(output)
    (output / "comparison.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    )
    with (output / "metrics.csv").open("w", newline="") as stream:
        if report["metrics"]:
            writer = csv.DictWriter(stream, fieldnames=list(report["metrics"][0]))
            writer.writeheader()
            writer.writerows(report["metrics"])


def main():
    parser = argparse.ArgumentParser(description="Compare recorded dual-scenario runs")
    parser.add_argument("--input", required=True, help="Run directory containing execution.json")
    parser.add_argument(
        "--output", required=True, help="New directory for comparison.json/metrics.csv"
    )
    args = parser.parse_args()
    root, output = (Path(p).expanduser().resolve() for p in (args.input, args.output))
    if output.is_relative_to(root):
        parser.error("Choose an output directory outside the input run directory")
    try:
        if output.exists():
            raise FileExistsError(output)
        report = summarize_runs(root)
        output.mkdir(parents=True, exist_ok=False)
        write_comparison(output, report)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(
        f"Status: {report['status']}; supported comparisons: "
        f"{report['benefit_supported_pairs']}/{report['requested_repetitions']}"
    )
    print(f"Results: {output}")
    return 0 if report["status"] == "COMPLETE" else 1
