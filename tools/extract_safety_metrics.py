#!/usr/bin/env python3
"""Compute offline TTC, TTLC and collision-speed proxies from recorded driving."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ksae_2026_autumn.safety_metrics_io import extract  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", required=True, help="Run directory containing routes/<route_id>/")
    p.add_argument("--output", required=True, help="New output directory outside the input run")
    args = p.parse_args()
    try:
        result = extract(args.input, args.output)
        print("Metric extraction:", result["status"])
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
