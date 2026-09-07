"""Run TransFuser++ through CARLA Garage's local evaluator."""

import argparse
from pathlib import Path

import yaml

from ksae_2026_autumn.tfpp import run


def main() -> int:
    parser = argparse.ArgumentParser(description="Run CARLA Garage's local TF++ evaluator")
    parser.add_argument(
        "--config", type=Path, default=Path("configs/tfpp.yaml"),
        help="configuration YAML (default: configs/tfpp.yaml)",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        help="new output directory; overrides output_dir in the configuration",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="validate file paths and print the command without connecting to CARLA",
    )
    args = parser.parse_args()
    try:
        return run(args.config, dry_run=args.dry_run, output_dir=args.output_dir)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
