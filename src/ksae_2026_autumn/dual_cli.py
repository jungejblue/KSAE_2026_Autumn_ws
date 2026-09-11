"""Run clean, held-command, and fallback-handover variants of a lead-stop scenario."""

import argparse
import json
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

from . import violation_launcher
from .dual_control import Settings
from .dual_results import summarize_runs, write_comparison
from .telemetry import write_json

ROOT = Path(__file__).resolve().parents[2]


def load_config(filename):
    path = Path(filename).expanduser().resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Keep the scenario config inside this repository for source capture")
    config = json.loads(path.read_text())
    if config.get("schema_version") != 1 or config.get("scenario") != "controlled_lead_stop":
        raise ValueError("Expected a controlled_lead_stop config with schema_version 1")
    if config.get("control_dt") != 0.05 or config.get("time_limit_ticks") != 1200:
        raise ValueError("This scenario requires 0.05 s ticks and a 1200-tick time limit")
    if config.get("protocol") != asdict(Settings()):
        raise ValueError("Unsupported protocol: use the documented fixed lead-stop settings")
    paths = {}
    for key in ("route_xml", "fallback_config"):
        relative = Path(config[key])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"{key} must be a relative path within the config directory")
        target = (path.parent / relative).resolve()
        if not target.is_relative_to(ROOT) or not target.is_file():
            raise ValueError(f"Missing or external configuration file: {target}")
        paths[key] = target
    routes = ET.parse(paths["route_xml"]).getroot().findall("route")
    if (
        len(routes) != 1
        or routes[0].get("id") != "70001"
        or routes[0].get("town") != "Town01"
        or list(routes[0].findall("scenarios/scenario"))
    ):
        raise ValueError("Expected one Town01 route 70001 with no other scenarios")
    lead = config["lead_spawn"]
    for key in ("x", "y", "z", "yaw", "road", "lane"):
        if isinstance(lead[key], bool) or not math.isfinite(float(lead[key])):
            raise ValueError(f"Invalid lead_spawn.{key}")
    fallback = json.loads(paths["fallback_config"].read_text())
    if fallback.get("backend") != "sampled" or fallback.get("control_dt") != 0.05:
        raise ValueError("This scenario uses the sampled fallback with control_dt=0.05")
    return path, config, paths


def run_suite(args):
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("Use Python 3.10 with the CARLA Garage dependencies")
    from fallback_control import Config

    config_path, config, paths = load_config(args.config)
    Config(**json.loads(paths["fallback_config"].read_text()))
    root = Path(args.output).expanduser().resolve()
    garage = Path(args.garage).expanduser().resolve()
    carla = Path(args.carla).expanduser().resolve()
    model = Path(args.model).expanduser().resolve()
    if any(root.is_relative_to(p) for p in (ROOT, garage, carla, model)):
        raise ValueError("Store outputs outside the repository, Garage, CARLA and model folders")
    common = dict(
        garage=str(garage),
        carla=str(carla),
        model=str(model),
        evaluator="b2d",
        routes=str(paths["route_xml"]),
        routes_subset="70001",
        logging="on",
        seed=100,
        gpu=args.gpu,
        host="localhost",
        port=args.port,
        tm_port=args.tm_port,
        timeout=args.timeout,
        delay_ms=None,
        delay_onset_s=5.0,
        delay_duration_s=2.0,
    )
    # Validate external paths/version before creating an execution directory.
    violation_launcher.prepare(SimpleNamespace(**common))
    root.mkdir(parents=True, exist_ok=False)
    saved_config = root / "config"
    saved_config.mkdir()
    shutil.copy2(config_path, saved_config / config_path.name)
    for key, path in paths.items():
        target = saved_config / config[key]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    extra = [str(p.relative_to(ROOT)) for p in (config_path, *paths.values())]
    extra.extend(
        str(p.relative_to(ROOT)) for p in sorted((ROOT / "src/fallback_control").glob("*.py"))
    )
    extra.extend(["tools/run_dual_scenario.py", "tools/compare_dual_runs.py"])
    write_json(
        root / "suite_config.json",
        {
            "schema_version": 1,
            "scenario": config["scenario"],
            "repetitions": args.repetitions,
            "windowed": args.windowed,
            "seed": 100,
            "protocol": config["protocol"],
            "scenario_config": config,
            "repository_commit": violation_launcher.git_output(ROOT, "rev-parse", "HEAD"),
            "fault_kind": "hold_last_command_compute_stall",
            "stall_s": 3.0,
            "takeover_after_s": 0.75,
            "horizon_s": 5.0,
        },
    )
    executions = []
    write_json(root / "execution.json", executions)
    stop = False
    for rep in range(args.repetitions):
        directory = root / f"rep_{rep:02d}"
        directory.mkdir()
        for mode in ("clean", "delayed", "takeover"):
            case_output = directory / mode
            item = {"repetition": rep, "mode": mode, "returncode": None}
            executions.append(item)
            write_json(root / "execution.json", executions)
            case = SimpleNamespace(**common, output=str(case_output))
            environment = {
                "KSAE_DUAL_MODE": mode,
                "KSAE_DUAL_WINDOWED": "1" if args.windowed else "0",
                "KSAE_DUAL_CONFIG": str(saved_config / config_path.name),
                "KSAE_DUAL_REFERENCE_TICKS": str(
                    directory / "delayed/routes/RouteScenario_70001_rep0/ticks.jsonl"
                ),
            }
            print(f"\nRepetition {rep + 1}/{args.repetitions}: {mode}", flush=True)
            try:
                item["returncode"] = violation_launcher.run(
                    case,
                    runtime_module="ksae_2026_autumn.dual_runtime",
                    runtime_environment=environment,
                    extra_sources=extra,
                )
            except (OSError, ValueError, RuntimeError, KeyError, KeyboardInterrupt) as exc:
                item.update(
                    returncode=130 if isinstance(exc, KeyboardInterrupt) else 1,
                    error=f"{type(exc).__name__}: {exc}",
                )
            write_json(root / "execution.json", executions)
            if item["returncode"] != 0:
                stop = True
                break
        if stop:
            break
    report = summarize_runs(root)
    write_comparison(root, report)
    print(f"\nResults: {root / 'comparison.json'}", flush=True)
    print(
        f"Supported comparisons: {report['benefit_supported_pairs']} / "
        f"{report['requested_repetitions']}",
        flush=True,
    )
    return 0 if report["status"] == "COMPLETE" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/dual_scenario.json"))
    parser.add_argument("--garage", default=os.environ.get("CARLA_GARAGE_ROOT"))
    parser.add_argument("--carla", default=os.environ.get("CARLA_ROOT"))
    parser.add_argument("--model", default=os.environ.get("TFPP_MODEL"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--repetitions", type=int, choices=range(1, 4), default=1)
    parser.add_argument("--windowed", action="store_true", help="Show the CARLA spectator window")
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    if not all((args.garage, args.carla, args.model)):
        parser.error("Set CARLA_GARAGE_ROOT, CARLA_ROOT, TFPP_MODEL or pass their options")
    if not 1 <= args.port <= 65532 or not 1 <= args.tm_port <= 65535:
        parser.error("Invalid CARLA or Traffic Manager port")
    if args.gpu < 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("GPU index must be nonnegative and timeout must be positive")
    try:
        return run_suite(args)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
