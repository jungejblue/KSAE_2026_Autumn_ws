#!/usr/bin/env python3
"""Check one completed telemetry run."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ksae_2026_autumn.telemetry_cli import main  # noqa: E402

if __name__ == "__main__":
    sys.argv.insert(1, "check")
    raise SystemExit(main())
