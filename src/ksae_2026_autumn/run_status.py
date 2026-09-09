"""Propagate evaluator and recorder failures to the caller."""

import json
from pathlib import Path


def require_finished(output, expected_routes, violations=False):
    output = Path(output)
    result = json.loads((output / "result.json").read_text())
    checkpoint = result.get("_checkpoint", {})
    records = checkpoint.get("records", [])
    actual = [record.get("route_id") for record in records]
    if (
        result.get("entry_status") != "Finished"
        or result.get("eligible") is not True
        or not expected_routes
        or len(actual) != len(expected_routes)
        or set(actual) != set(expected_routes)
        or checkpoint.get("progress") != [len(expected_routes), len(expected_routes)]
    ):
        raise RuntimeError("Evaluator did not finish all requested routes")
    for record in records:
        status = str(record.get("status", "")).lower()
        if not status or any(
            word in status for word in ("crash", "couldn't", "invalid", "started")
        ):
            raise RuntimeError(f"Evaluator failure: {record}")
    for route_id in expected_routes:
        names = ["summary.json"]
        if violations:
            names.append("violation_summary.json")
        for name in names:
            path = output / "routes" / route_id / name
            summary = json.loads(path.read_text())
            if (
                not summary.get("closed")
                or summary.get("close_reason") != "stop_scenario"
                or summary.get("errors")
                or (violations and name == "violation_summary.json" and not summary.get("rows"))
            ):
                raise RuntimeError(f"Recorder did not finish normally: {path}")
