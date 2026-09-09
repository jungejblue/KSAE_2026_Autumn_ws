"""Run TransFuser++ with telemetry and collision/route-departure recording."""

import argparse
import hashlib
import json
import os
import shlex
import shutil
import socket
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

from ksae_2026_autumn.run_status import require_finished
from ksae_2026_autumn.telemetry import write_json

ROOT = Path(__file__).resolve().parents[2]
GARAGE_COMMIT = "72f39a63423a5edef6904b1487e0360a64bcf445"
SUPPORTED = {
    "team_code/sensor_agent.py": "2dfb3932a73ed7f2074e622d252a312e81ccc8ec",
    "Bench2Drive/leaderboard/leaderboard/scenarios/scenario_manager.py": (
        "f71b4753f5829c63683dba35b97c472b4338dbc1"
    ),
    "Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py": (
        "0aa615e8795fafdf22449186dd1016e0438c5576"
    ),
    "Bench2Drive/leaderboard/leaderboard/envs/sensor_interface.py": (
        "7fd94752f62d832f6be4bc6da3398c3b46f2597c"
    ),
    "leaderboard/leaderboard/scenarios/scenario_manager.py": (
        "934dfb7d7e1a9c5b8345653f487909d7b0be2d56"
    ),
    "leaderboard/leaderboard/leaderboard_evaluator_local.py": (
        "eda98e5e26ec9548cf53f9c22614d4e684b08c24"
    ),
}
CONTROL_ENV = {
    "CHALLENGE_TRACK_CODENAME": "SENSORS",
    "DEBUG_CHALLENGE": "0",
    "DIRECT": "1",
    "UNCERTAINTY_WEIGHT": "1",
    "UNCERTAINTY_THRESHOLD": "0.9",
    "TUNED_AIM_DISTANCE": "0",
    "SLOWER": "0",
    "STOP_CONTROL": "1",
    "STOP_AFTER_METER": "-1",
    "COMPILE": "0",
}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_blob(path):
    data = Path(path).read_bytes()
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def git_output(root, *args):
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), *args], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def implementation_hashes():
    files = sorted((ROOT / "src/ksae_2026_autumn").glob("*.py"))
    return {str(p.relative_to(ROOT)): sha256(p) for p in files}


def prepare(args):
    garage, carla, model = (
        Path(p).expanduser().resolve() for p in (args.garage, args.carla, args.model)
    )
    base = garage / "Bench2Drive" if args.evaluator == "b2d" else garage
    evaluator = (
        base
        / "leaderboard/leaderboard"
        / (
            "leaderboard_evaluator.py"
            if args.evaluator == "b2d"
            else "leaderboard_evaluator_local.py"
        )
    )
    routes = (
        Path(args.routes).expanduser().resolve()
        if args.routes
        else base
        / "leaderboard/data"
        / ("bench2drive220.xml" if args.evaluator == "b2d" else "debug.xml")
    )
    if args.evaluator == "b2d" and not (carla / "CarlaUE4.sh").is_file():
        raise FileNotFoundError(carla / "CarlaUE4.sh")
    if not (carla / "PythonAPI/carla").is_dir():
        raise FileNotFoundError(carla / "PythonAPI/carla")
    if not (model / "config.json").is_file():
        raise FileNotFoundError(model / "config.json")
    weights = sorted(model.glob("*.pth"))
    if len(weights) != 1:
        raise ValueError(
            "Model directory must contain exactly one .pth file and matching config.json"
        )
    critical = [
        garage / "team_code/sensor_agent.py",
        evaluator,
        base / "leaderboard/leaderboard/scenarios/scenario_manager.py",
    ]
    if args.evaluator == "b2d":
        critical.append(base / "leaderboard/leaderboard/envs/sensor_interface.py")
    hashes = {}
    for path in critical:
        relative = str(path.relative_to(garage))
        value = git_blob(path)
        if value != SUPPORTED[relative]:
            raise ValueError(
                f"Unsupported Garage file: {relative}\nExpected source from {GARAGE_COMMIT}; "
                "Use the supported Garage checkout for this adapter."
            )
        hashes[relative] = value
    route_elements = ET.parse(routes).getroot().findall("route")
    all_ids = [r.attrib["id"] for r in route_elements]
    if args.evaluator == "b2d":
        selected = args.routes_subset.split(",")
        if (
            not selected
            or any(not x or x not in all_ids for x in selected)
            or len(selected) != len(set(selected))
        ):
            raise ValueError(
                "--routes-subset must be unique comma-separated route IDs present in the XML"
            )
        selected = [r for r in all_ids if r in selected]
    else:
        if args.routes_subset:
            raise ValueError(
                "The local evaluator runs the whole supplied XML; omit --routes-subset"
            )
        selected = all_ids
    if not selected:
        raise ValueError("No routes selected")
    return garage, carla, model, base, evaluator, routes, weights[0], selected, hashes


def run(args):
    if args.logging != "on":
        raise ValueError("Violation recording requires logging on")
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("Use Python 3.10 with CARLA Garage dependencies installed")
    garage, carla, model, base, evaluator, routes, weight, selected, hashes = prepare(args)
    output = Path(args.output).expanduser().resolve()
    # Refuse to mix an earlier execution into this one, even when it failed.
    source_hashes = implementation_hashes()
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(CONTROL_ENV)
    env.pop("SAVE_PATH", None)
    env.update(
        {
            "CARLA_ROOT": str(carla),
            "WORK_DIR": str(base),
            "SCENARIO_RUNNER_ROOT": str(base / "scenario_runner"),
            "LEADERBOARD_ROOT": str(base / "leaderboard"),
            "TEAM_CODE_ROOT": str(garage / "team_code"),
            "PYTHONPATH": os.pathsep.join(
                map(
                    str,
                    [
                        ROOT / "src",
                        carla / "PythonAPI/carla",
                        base / "scenario_runner",
                        base / "leaderboard",
                        garage / "team_code",
                    ],
                )
            ),
            "PYTHONNOUSERSITE": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONUNBUFFERED": "1",
            "IS_BENCH2DRIVE": "True" if args.evaluator == "b2d" else "False",
            "CUDA_VISIBLE_DEVICES": str(args.gpu),
            "KSAE_TELEMETRY_OUTPUT": str(output),
        }
    )
    arguments = [
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--traffic-manager-port",
        str(args.tm_port),
        "--traffic-manager-seed",
        str(args.seed),
        "--routes",
        str(routes),
        "--repetitions",
        "1",
        "--track",
        "SENSORS",
        "--checkpoint",
        str(output / "result.json"),
        "--debug-checkpoint",
        str(output / "live_results.txt"),
        "--agent",
        str(ROOT / "src/ksae_2026_autumn/logging_agent.py"),
        "--agent-config",
        str(model),
        "--debug",
        "0",
        "--timeout",
        str(args.timeout),
    ]
    if args.evaluator == "b2d":
        arguments += ["--routes-subset", ",".join(selected), "--gpu-rank", str(args.gpu)]
    else:
        arguments += ["--resume", "0"]
    command = [
        sys.executable,
        "-u",
        "-m",
        "ksae_2026_autumn.violation_runtime",
        str(evaluator),
        *arguments,
    ]
    meta = {
        "schema_version": 1,
        "run_id": str(uuid.uuid4()),
        "started_at": utc_now(),
        "logging_enabled": args.logging == "on",
        "evaluator": args.evaluator,
        "seed": args.seed,
        "expected_route_ids": [f"RouteScenario_{r}_rep0" for r in selected],
        "command": command,
        "working_directory": str(base),
        "garage_root": str(garage),
        "carla_root": str(carla),
        "model_dir": str(model),
        "garage_commit": git_output(garage, "rev-parse", "HEAD"),
        "research_commit": git_output(ROOT, "rev-parse", "HEAD"),
        "model_sha256": sha256(weight),
        "config_sha256": sha256(model / "config.json"),
        "routes_sha256": sha256(routes),
        "garage_source_hashes": hashes,
        "implementation_hashes": source_hashes,
        "control_environment": CONTROL_ENV,
        "pythonpath": env["PYTHONPATH"],
        "cuda_visible_devices": env["CUDA_VISIBLE_DEVICES"],
        "sensor_timestamp_note": (
            "Sensor tuples retain frame IDs; world and agent clocks have different origins."
        ),
        "action_timestamp_note": (
            "No injected delay; native multi-frame LiDAR history is recorded separately."
        ),
    }
    write_json(output / "run.json", meta)
    source = output / "source"
    for relative in meta["implementation_hashes"]:
        destination = source / "research" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, destination)
    for relative in hashes:
        destination = source / "garage" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(garage / relative, destination)
    shutil.copy2(model / "config.json", source / "model_config.json")
    shutil.copy2(routes, source / "routes.xml")
    for name, root in (("research", ROOT), ("garage", garage)):
        (source / f"{name}_status.txt").write_text(
            (git_output(root, "status", "--short") or "") + "\n"
        )
    try:
        connection = socket.create_connection((args.host, args.port), timeout=1)
    except OSError:
        connection = None
    if connection:
        connection.close()
    if args.evaluator == "b2d" and connection is not None:
        meta.update(exit_code=2, finished_at=utc_now(), error="CARLA port already in use")
        write_json(output / "run.json", meta)
        (output / "exit_code.txt").write_text("2\n")
        raise RuntimeError(
            "Stop the existing CARLA server with Ctrl+C first. Bench2Drive starts its own server. "
            "Use a fresh output directory for retry."
        )
    print(shlex.join(command), flush=True)
    print(f"Output: {output}", flush=True)
    code = 1
    try:
        with (output / "run.log").open("x", encoding="utf-8") as log:
            with subprocess.Popen(
                command,
                env=env,
                cwd=base,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
                bufsize=1,
            ) as process:
                try:
                    for line in process.stdout:
                        sys.stdout.write(line)
                        log.write(line)
                        log.flush()
                    code = process.wait()
                except KeyboardInterrupt:
                    # The terminal also delivers SIGINT to the child/evaluator.
                    try:
                        code = process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        code = process.wait(timeout=10)
                    if code == 0:
                        code = 130
    finally:
        latest = json.loads((output / "run.json").read_text())
        latest.update(exit_code=code, finished_at=utc_now())
        write_json(output / "run.json", latest)
        (output / "exit_code.txt").write_text(str(code) + "\n")
    if code != 0:
        return code if code > 0 else 1
    try:
        require_finished(output, meta["expected_route_ids"], violations=True)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        latest = json.loads((output / "run.json").read_text())
        latest.update(completion_error=str(error), launcher_exit_code=1)
        write_json(output / "run.json", latest)
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"Run completed. Results: {output}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run_parser = sub.add_parser("run")
    run_parser.add_argument(
        "--garage",
        default=os.environ.get(
            "CARLA_GARAGE_ROOT",
            os.environ.get("GARAGE_ROOT", str(Path.home() / "carla_garage")),
        ),
    )
    run_parser.add_argument(
        "--carla", default=os.environ.get("CARLA_ROOT", str(Path.home() / "e2e_carla_ws"))
    )
    run_parser.add_argument("--model", default=os.environ.get("TFPP_MODEL"))
    run_parser.add_argument("--evaluator", choices=["b2d", "local"], default="b2d")
    run_parser.add_argument("--logging", choices=["on"], default="on")
    run_parser.add_argument("--routes-subset", default="")
    run_parser.add_argument("--routes")
    run_parser.add_argument("--output", required=True)
    run_parser.add_argument("--seed", type=int, default=100)
    run_parser.add_argument("--host", default="localhost")
    run_parser.add_argument("--port", type=int, default=2000)
    run_parser.add_argument("--tm-port", type=int, default=8000)
    run_parser.add_argument("--gpu", type=int, default=0)
    run_parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    try:
        if not args.model:
            parser.error("set TFPP_MODEL or pass --model")
        return run(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
