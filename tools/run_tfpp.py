"""Run from an environment where this repository is installed with pip install -e ."""

import argparse
from pathlib import Path

from ksae_2026_autumn.tfpp import run


def main() -> int:
    parser = argparse.ArgumentParser(description="Run CARLA Garage's local TF++ evaluator")
    parser.add_argument("--config", type=Path, default=Path("configs/tfpp.yaml"))
    parser.add_argument(
        "--dry-run", action="store_true", help="validate paths and print the command"
    )
    args = parser.parse_args()
    try:
        return run(args.config, dry_run=args.dry_run)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
