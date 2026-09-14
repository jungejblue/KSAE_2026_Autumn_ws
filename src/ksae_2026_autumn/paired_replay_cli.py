"""Run and review independent continuations after a reconstructed candidate."""

import argparse
import json
import math
import os
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

from . import violation_launcher
from .dual_cli import load_config
from .paired_replay_review import CASES, analyze_case, review
from .telemetry import write_json

ROOT = Path(__file__).resolve().parents[2]


def save_review(root):
    report = review(root)
    write_json(Path(root) / "review.json", report)
    print(f"Comparison: {report['status']}")
    return report


def run(args):
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("Use Python 3.10 with CARLA Garage dependencies")
    from fallback_control import Config

    path, config, paths = load_config(ROOT / "configs/dual_scenario.json")
    Config(**json.loads(paths["fallback_config"].read_text()))
    root = Path(args.output).expanduser().resolve()
    garage, carla, model = [
        Path(p).expanduser().resolve() for p in (args.garage, args.carla, args.model)
    ]
    if any(root.is_relative_to(p) for p in (ROOT, garage, carla, model)):
        raise ValueError("Keep outputs outside the repository, Garage, CARLA and model folders")
    common = dict(
        garage=str(garage),
        carla=str(carla),
        model=str(model),
        evaluator="b2d",
        routes=str(paths["route_xml"]),
        routes_subset="70001",
        logging="on",
        seed=args.seed,
        gpu=args.gpu,
        host="localhost",
        port=args.port,
        tm_port=args.tm_port,
        timeout=args.timeout,
        delay_ms=None,
        delay_onset_s=5.0,
        delay_duration_s=2.0,
    )
    violation_launcher.prepare(SimpleNamespace(**common))
    root.mkdir(parents=True, exist_ok=False)
    saved_config = root / "config"
    saved_config.mkdir()
    shutil.copy2(path, saved_config / path.name)
    for key, source in paths.items():
        target = saved_config / config[key]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    extras = [str(p.relative_to(ROOT)) for p in (path, *paths.values())]
    extras += [
        str(p.relative_to(ROOT)) for p in sorted((ROOT / "src/fallback_control").glob("*.py"))
    ]
    extras += ["tools/run_paired_replay.py"]
    # Retain all TF++ Python dependencies, not only its agent entry point.
    dependencies = {}
    for source in sorted((garage / "team_code").glob("*.py")):
        dependencies[source.name] = violation_launcher.sha256(source)
        target = root / "garage_team_code" / source.name
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(source, target)
    write_json(
        root / "replay_suite.json",
        {
            "schema_version": 1,
            "repetitions": args.repetitions,
            "seed": args.seed,
            "cases": list(CASES),
            "horizon_s": 3,
            "candidate_after_fault_s": 0.75,
            "fault_duration_s": 3,
            "fault_kind": "hold_last_command_compute_stall",
            "method": "common_candidate_input_live_continuation",
            "post_candidate_command_tolerance": None,
            "backend": "fixed",
            "full_snapshot_restore": False,
            "repository_commit": violation_launcher.git_output(ROOT, "rev-parse", "HEAD"),
            "scenario_config": config,
            "effective_replay_settings": {
                **config["protocol"], "horizon_ticks": 60,
            },
            "garage_team_code_sha256": dependencies,
            "note": (
                "scenario_config describes the shared scenario; "
                "effective_replay_settings records the 3-second evaluation horizon."
            ),
        },
    )
    attempts = []
    write_json(root / "execution.json", attempts)
    stop = False
    for rep in range(args.repetitions):
        directory = root / f"rep_{rep:02d}"
        directory.mkdir()
        for name in CASES:
            item = {"repetition": rep, "case": name, "returncode": None}
            attempts.append(item)
            write_json(root / "execution.json", attempts)
            env = {
                "KSAE_REPLAY_CASE": name,
                "KSAE_REPLAY_SEED": str(args.seed),
                "KSAE_DUAL_WINDOWED": "0",
                "KSAE_DUAL_CONFIG": str(saved_config / path.name),
                "KSAE_DUAL_REFERENCE_TICKS": str(
                    directory / "E0/routes/RouteScenario_70001_rep0/ticks.jsonl"
                ),
                "PYTHONHASHSEED": str(args.seed),
                "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
            }
            print(f"Repetition {rep + 1}/{args.repetitions}, branch {name}", flush=True)
            try:
                item["returncode"] = violation_launcher.run(
                    SimpleNamespace(**common, output=str(directory / name)),
                    runtime_module="ksae_2026_autumn.paired_replay_runtime",
                    runtime_environment=env,
                    extra_sources=extras,
                )
            except (
                OSError,
                RuntimeError,
                ValueError,
                KeyError,
                KeyboardInterrupt,
            ) as exc:
                item.update(
                    returncode=130 if isinstance(exc, KeyboardInterrupt) else 1,
                    error=f"{type(exc).__name__}: {exc}",
                )
            write_json(root / "execution.json", attempts)
            case = analyze_case(directory / name, "takeover" if name.startswith("F") else "delayed")
            write_json(directory / (name + "_case_review.json"), case)
            if item["returncode"] != 0 or case["status"] == "FAIL":
                stop = True
                break
        report = save_review(root)
        if stop or report["repetitions"][str(rep)]["status"] != "PASS":
            break
    # Check the external Python dependency source has not changed during the suite.
    after = {
        p.name: violation_launcher.sha256(p) for p in sorted((garage / "team_code").glob("*.py"))
    }
    write_json(root / "garage_sources_after.json", after)
    report = save_review(root)
    if dependencies != after:
        report.update(
            status="FAIL",
            error="Garage team_code sources changed during execution",
        )
        write_json(root / "review.json", report)
    return 0 if report["status"] == "PASS" else 1 if report["status"] == "FAIL" else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run_parser = sub.add_parser("run")
    run_parser.add_argument("--garage", default=os.environ.get("CARLA_GARAGE_ROOT"))
    run_parser.add_argument("--carla", default=os.environ.get("CARLA_ROOT"))
    run_parser.add_argument("--model", default=os.environ.get("TFPP_MODEL"))
    run_parser.add_argument("--output", required=True)
    run_parser.add_argument("--repetitions", type=int, choices=(1, 3), default=1)
    run_parser.add_argument("--seed", type=int, default=100)
    run_parser.add_argument("--gpu", type=int, default=0)
    run_parser.add_argument("--port", type=int, default=2000)
    run_parser.add_argument("--tm-port", type=int, default=8000)
    run_parser.add_argument("--timeout", type=float, default=600)
    check = sub.add_parser("review")
    check.add_argument("--input", required=True)
    check.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        if args.action == "run":
            if not all((args.garage, args.carla, args.model)):
                parser.error("Set CARLA_GARAGE_ROOT, CARLA_ROOT, TFPP_MODEL or pass options")
            if not (
                0 <= args.seed < 2**32
                and args.gpu >= 0
                and 1 <= args.port <= 65532
                and 1 <= args.tm_port <= 65535
                and args.tm_port not in range(args.port, args.port + 3)
                and math.isfinite(args.timeout)
                and args.timeout > 0
            ):
                parser.error("Invalid seed, GPU, ports, or timeout")
            return run(args)
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_relative_to(Path(args.input).expanduser().resolve()):
            raise ValueError("Choose a new review output outside the input directory")
        report = review(args.input)
        output.mkdir(parents=True, exist_ok=False)
        write_json(output / "review.json", report)
        print(f"Comparison: {report['status']}")
        return 0 if report["status"] == "PASS" else 2
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
