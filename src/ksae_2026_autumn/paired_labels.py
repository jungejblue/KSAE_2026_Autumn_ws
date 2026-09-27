"""Build outcome labels and auxiliary metrics from audited paired replay records."""

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

from .paired_replay_review import review, route
from .safety_metrics_io import compute_route, read_jsonl, write_csv, write_json

OUTCOMES = {
    (False, False): "UNNECESSARY",
    (True, False): "NECESSARY_EFFECTIVE",
    (True, True): "NECESSARY_INEFFECTIVE",
    (False, True): "HARMFUL",
}


def classify(e, f):
    if type(e) is not bool or type(f) is not bool:
        raise ValueError("Outcome inputs must be booleans")
    return OUTCOMES[e, f]


def time_summary(rows, key):
    statuses = Counter(r[key + "_status"] for r in rows)
    finite = [r[key + "_s"] for r in rows if r[key + "_s"] is not None]
    allowed = {
        "ttc": {"predicted_overlap", "no_overlap_within_horizon"},
        "ttlc": {"already_outside", "predicted_exit", "no_exit_on_prediction_grid"},
    }
    known = not (set(statuses) - allowed[key])
    # This is a capped prediction metric, not an observed time-to-event.
    capped = min(3.0 if r[key + "_s"] is None else r[key + "_s"] for r in rows) if known else None
    return {
        "finite_min_s": min(finite) if finite else None,
        "capped_min_s": capped,
        "status_counts": dict(statuses),
        "coverage_known": known,
    }


def branch_metrics(folder, candidate, target):
    all_rows, all_speeds, _ = compute_route(folder)
    ticks = read_jsonl(folder / "ticks.jsonl")
    window = all_rows[candidate : candidate + 61]
    if len(window) != 61:
        raise ValueError("Incomplete outcome window")
    start, stop = window[0]["frame"], window[-1]["frame"]
    speeds = [r for r in all_speeds if start <= r["frame"] <= stop]
    target.mkdir(parents=True)
    write_csv(target / "metrics.csv", window)
    write_csv(target / "collision_speeds.csv", speeds)
    collision = any(r["collision_at_frame"] for r in window)
    lane = any(r["lane_outside_at_frame"] for r in window)
    events = [
        i * 0.05
        for i, r in enumerate(window)
        if r["collision_at_frame"] or r["lane_outside_at_frame"]
    ]
    result = {
        "collision": collision,
        "lane": lane,
        "V_H": collision or lane,
        "window_frames": len(window),
        "horizon_s": 3.0,
        "first_violation_s": events[0] if events else None,
        "lane_departure_frames": sum(r["lane_outside_at_frame"] for r in window),
        "ttc": time_summary(window, "ttc"),
        "ttlc": time_summary(window, "ttlc"),
        "collision_callbacks": len(speeds),
        "first_collision_speed_proxy_mps": speeds[0]["collision_speed_proxy_mps"]
        if speeds
        else None,
        "collision_speed_status": speeds[0]["status"] if speeds else "no_collision",
        "candidate_speed_mps": ticks[candidate]["ego"]["speed_mps"],
    }
    write_json(target / "summary.json", result)
    return result


def benefits(e, f):
    out = {}
    for key in ("ttc", "ttlc"):
        a, b = e[key]["capped_min_s"], f[key]["capped_min_s"]
        out["delta_" + key + "_capped_min_s"] = None if a is None or b is None else b - a
    a, b = e["first_collision_speed_proxy_mps"], f["first_collision_speed_proxy_mps"]
    out["collision_speed_reduction_proxy_mps"] = (
        a - b if e["collision"] and f["collision"] and a is not None and b is not None else None
    )
    return out


def input_hashes(root):
    return {
        str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(root.rglob("*"))
        if p.is_file()
    }


def build(source, output):
    source, output = Path(source).expanduser().resolve(), Path(output).expanduser().resolve()
    repo = Path(__file__).resolve().parents[2]
    if (
        output.is_relative_to(source)
        or output.is_relative_to(repo)
        or source.is_relative_to(output)
    ):
        raise ValueError("Use a new output folder outside both source and repository")
    output.mkdir(parents=True, exist_ok=False)
    status = {
        "status": "FAIL",
        "scope": "controlled_fixture_label_pipeline",
        "dev_dataset_complete": False,
        "monitor_evaluation_performed": False,
    }
    try:
        hashes = input_hashes(source)
        stored = source / "SHA256SUMS.json"
        if stored.exists():
            for name, expected in json.loads(stored.read_text()).items():
                if hashes.get(name) != expected:
                    raise ValueError("Source hash mismatch: " + name)
        write_json(output / "source_hashes.json", hashes)
        r = review(source)
        write_json(output / "recomputed_replay_review.json", r)
        if r["status"] == "FAIL":
            raise ValueError("Replay audit contains execution or dependency errors")
        if not r["garage_dependency_check_complete"]:
            raise ValueError("Missing end-of-suite dependency check")
        records = []
        suite_id = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:16]
        for rep, group in sorted(r["repetitions"].items()):
            event = {
                "event_id": f"{suite_id}_rep_{int(rep):02d}",
                "repetition": int(rep),
                "candidate_rule": "fault_onset_plus_0.75s",
                "horizon_s": 3.0,
                "decision_intervene": None,
                "valid_pair": group["status"] == "PASS",
                "representative_branches": ["E1", "F1"],
            }
            records.append(event)
            if not event["valid_pair"]:
                event.update(
                    outcome="INVALID_PAIR", invalid_reason=group.get("error", group["status"])
                )
                continue
            candidate = group["candidate_tick"]
            base = source / f"rep_{int(rep):02d}"
            meta = json.loads((base / "E1/run.json").read_text())
            event.update(candidate_tick=candidate, seed=meta["seed"], route_id="70001")
            branches = {}
            for name in ("E1", "F1"):
                b = branch_metrics(
                    route(base / name), candidate, output / "metrics" / event["event_id"] / name
                )
                if b["V_H"] != group["cases"][name]["V_H"]:
                    raise ValueError("Metric/replay violation disagreement")
                branches[name] = b
            e, f = branches["E1"], branches["F1"]
            event.update(
                branch_e=e,
                branch_f=f,
                necessity=e["V_H"],
                effective_avoidance=e["V_H"] and not f["V_H"],
                outcome=classify(e["V_H"], f["V_H"]),
                benefits=benefits(e, f),
            )
        write_json(output / "paired_events.json", records)
        flat = []
        for e in records:
            row = {
                k: e.get(k)
                for k in (
                    "event_id",
                    "repetition",
                    "seed",
                    "route_id",
                    "candidate_tick",
                    "horizon_s",
                    "valid_pair",
                    "outcome",
                    "necessity",
                )
            }
            for key in ("branch_e", "branch_f"):
                row[key + "_V_H"] = e.get(key, {}).get("V_H")
            row.update(
                {
                    k: e.get("benefits", {}).get(k)
                    for k in (
                        "delta_ttc_capped_min_s",
                        "delta_ttlc_capped_min_s",
                        "collision_speed_reduction_proxy_mps",
                    )
                }
            )
            flat.append(row)
        with (output / "paired_events.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
            writer.writeheader()
            writer.writerows(flat)
        valid = [e for e in records if e["valid_pair"]]
        unknown = [
            e["event_id"]
            for e in valid
            if any(
                not e[b][m]["coverage_known"]
                for b in ("branch_e", "branch_f")
                for m in ("ttc", "ttlc")
            )
        ]
        status.update(
            status="PASS" if len(valid) == len(records) and valid and not unknown else "REVIEW",
            replay_status=r["status"],
            total_events=len(records),
            valid_events=len(valid),
            invalid_events=len(records) - len(valid),
            unknown_metric_events=unknown,
            outcomes=dict(Counter(e["outcome"] for e in valid)),
            count_unit="one candidate per repetition; E0/E2/F2 are audit repeats",
            confusion=None,
            source=str(source),
        )
        if input_hashes(source) != hashes:
            raise ValueError("Source files changed during analysis")
    except Exception as exc:
        status.update(status="FAIL", error=f"{type(exc).__name__}: {exc}")
    finally:
        write_json(output / "label_review.json", status)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = build(args.input, args.output)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return {"PASS": 0, "REVIEW": 2, "FAIL": 1}[result["status"]]
