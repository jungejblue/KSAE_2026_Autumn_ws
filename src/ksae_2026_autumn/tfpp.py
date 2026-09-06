"""Launch Garage's local TF++ evaluator against an already running CARLA server."""

import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml


def expand_path(value: str) -> Path:
    expanded = os.path.expandvars(value)
    if re.search(r"\$\{?\w+", expanded):
        raise ValueError(f"set the environment variable referenced by: {value}")
    return Path(expanded).expanduser().resolve()


def prepare_run(config_path: Path) -> tuple[list[str], dict[str, str], dict]:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("config must be a YAML mapping")
    required = {"garage_root", "carla_root", "model_dir", "routes", "output_dir"}
    optional = {
        "host",
        "port",
        "traffic_manager_port",
        "traffic_manager_seed",
        "repetitions",
        "timeout",
        "debug",
    }
    if required - config.keys() or config.keys() - required - optional:
        raise ValueError("missing or unsupported config keys; use configs/tfpp.yaml")
    garage = expand_path(config["garage_root"])
    carla = expand_path(config["carla_root"])
    model = expand_path(config["model_dir"])
    output = expand_path(config["output_dir"])
    routes = Path(os.path.expandvars(config["routes"])).expanduser()
    routes = routes.resolve() if routes.is_absolute() else (garage / routes).resolve()
    evaluator = garage / "leaderboard/leaderboard/leaderboard_evaluator_local.py"
    agent = garage / "team_code/sensor_agent.py"
    for path in (evaluator, agent, routes, model / "config.json"):
        if not path.is_file():
            raise FileNotFoundError(path)
    weights = sorted(model.glob("*.pth"))
    if len(weights) != 1:
        raise ValueError("model_dir must contain exactly one .pth file to avoid implicit ensembles")
    for path in (carla / "PythonAPI/carla", garage / "scenario_runner"):
        if not path.is_dir():
            raise FileNotFoundError(path)
    # The Bench2Drive evaluator has different scenario behavior and owns server startup.
    # Do not silently run its routes through the local Leaderboard evaluator.
    if "Bench2Drive" in routes.parts or routes.name.startswith("bench2drive"):
        raise ValueError("use Garage's Bench2Drive evaluator for benchmark routes; see README")

    values = {
        "host": config.get("host", "127.0.0.1"),
        "port": config.get("port", 2000),
        "traffic-manager-port": config.get("traffic_manager_port", 8000),
        "traffic-manager-seed": config.get("traffic_manager_seed", 100),
        "repetitions": config.get("repetitions", 1),
        "timeout": config.get("timeout", 300),
        "debug": config.get("debug", 0),
        "routes": str(routes),
        "agent": str(agent),
        "agent-config": str(model),
        "checkpoint": str(output / "result.json"),
        "debug-checkpoint": str(output / "live_results.txt"),
        "track": "SENSORS",
        "resume": 0,
    }
    command = [sys.executable, str(evaluator)]
    for name, value in values.items():
        command.extend([f"--{name}", str(value)])
    env = os.environ.copy()
    # Replace inherited benchmark paths to prevent mixing two versions of leaderboard.
    env.update(
        {
            "CARLA_ROOT": str(carla),
            "WORK_DIR": str(garage),
            "SCENARIO_RUNNER_ROOT": str(garage / "scenario_runner"),
            "LEADERBOARD_ROOT": str(garage / "leaderboard"),
            "TEAM_CODE_ROOT": str(garage / "team_code"),
            "PYTHONPATH": os.pathsep.join(
                str(path)
                for path in (
                    carla / "PythonAPI/carla",
                    garage / "scenario_runner",
                    garage / "leaderboard",
                    garage / "team_code",
                )
            ),
            "IS_BENCH2DRIVE": "False",
            "STOP_AFTER_METER": "-1",
            "DEBUG_CHALLENGE": "0",
            "SAVE_PATH": str(output / "logs"),
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    resolved = {
        "garage_root": str(garage),
        "carla_root": str(carla),
        "model_dir": str(model),
        "output_dir": str(output),
        "weight_file": str(weights[0]),
        "arguments": values,
    }
    return command, env, resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(config_path: Path, dry_run: bool = False) -> int:
    command, env, resolved = prepare_run(config_path)
    print(shlex.join(command), flush=True)
    if dry_run:
        return 0
    output = Path(resolved["output_dir"])
    output.mkdir(parents=True, exist_ok=False)
    garage = Path(resolved["garage_root"])
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(garage), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    metadata = {
        "started_at": datetime.now(timezone.utc).isoformat(),
        "evaluator": "garage_local_debug",
        "garage_commit": commit,
        "weight_sha256": sha256_file(Path(resolved["weight_file"])),
        "model_config_sha256": sha256_file(Path(resolved["model_dir"]) / "config.json"),
        "routes_sha256": sha256_file(Path(resolved["arguments"]["routes"])),
        "config": resolved,
        "command": command,
    }
    metadata_path = output / "run.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    result = subprocess.run(command, env=env, cwd=garage, check=False)
    metadata["exit_code"] = result.returncode
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return result.returncode
