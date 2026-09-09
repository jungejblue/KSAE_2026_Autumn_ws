#!/usr/bin/env python3
"""Repository entry point; no editable installation required."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ksae_2026_autumn.violation_launcher import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
