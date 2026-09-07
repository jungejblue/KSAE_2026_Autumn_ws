"""Compare saved primary/fallback results and report classifications and metrics."""

import argparse
import json
from pathlib import Path

from ksae_2026_autumn.evaluation import evaluate_pairs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSON list; see configs/pairs.example.json")
    parser.add_argument("--output", type=Path, help="new output JSON path; defaults to stdout")
    parser.add_argument(
        "--horizon-seconds", type=float, default=3.0,
        help="required duration_s for both branches in every valid pair (default: 3.0)",
    )
    args = parser.parse_args()
    try:
        records = json.loads(args.input.read_text(encoding="utf-8"))
        result = evaluate_pairs(records, horizon_s=args.horizon_seconds)
        text = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(text)
            print(args.output)
        else:
            print(text, end="")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
