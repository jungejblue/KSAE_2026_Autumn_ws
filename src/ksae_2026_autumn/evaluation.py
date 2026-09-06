"""Score branch summaries already collected by an external paired execution runner."""

import math
from collections import Counter
from collections.abc import Iterable

OUTCOMES = {
    (False, False): "UNNECESSARY",
    (True, False): "NECESSARY_EFFECTIVE",
    (True, True): "NECESSARY_INEFFECTIVE",
    (False, True): "HARMFUL",
}


def require_bool(value: object) -> bool:
    if type(value) is not bool:
        raise ValueError("expected a JSON boolean (true/false), not a string or number")
    return value


def classify_pair(violation_e: bool, violation_f: bool) -> str:
    return OUTCOMES[(require_bool(violation_e), require_bool(violation_f))]


def classify_confusion(decision_intervene: bool, outcome: str) -> str:
    if outcome not in OUTCOMES.values():
        raise ValueError(f"unknown outcome: {outcome}")
    positive = outcome == "NECESSARY_EFFECTIVE"
    if require_bool(decision_intervene):
        return "TP" if positive else "FP"
    return "FN" if positive else "TN"


def any_wheel_outside_corridor(wheel_inside: Iterable[bool]) -> bool:
    states = tuple(wheel_inside)
    if len(states) != 4:
        raise ValueError("exactly four wheel states are required")
    return not all([require_bool(state) for state in states])


def capped_time_benefit(value_e_s: float, value_f_s: float, horizon_s: float = 3.0) -> float:
    if not math.isfinite(horizon_s) or horizon_s <= 0:
        raise ValueError("horizon must be finite and positive")
    if any(math.isnan(v) or v < 0 for v in (value_e_s, value_f_s)):
        raise ValueError("time values must be non-negative and not NaN")
    return min(value_f_s, horizon_s) - min(value_e_s, horizon_s)


def collision_speed_benefit(speed_e_mps: float, speed_f_mps: float) -> float:
    """Use only when both branches collide."""
    if any(not math.isfinite(v) or v < 0 for v in (speed_e_mps, speed_f_mps)):
        raise ValueError("speeds must be finite and non-negative")
    return speed_e_mps - speed_f_mps


def branch_violation(branch: dict, horizon_s: float) -> bool:
    duration = branch["duration_s"]
    if type(duration) not in (float, int) or not math.isclose(
        duration, horizon_s, rel_tol=0, abs_tol=1e-6
    ):
        raise ValueError("branch duration must match the labeling horizon")
    collision = require_bool(branch["collision"])
    lane = require_bool(branch["lane"])
    return collision or lane


def evaluate_pairs(records: list[dict], horizon_s: float = 3.0) -> dict:
    """Keep invalid pairs out of all confusion/outcome counts; report their reasons."""
    if not math.isfinite(horizon_s) or horizon_s <= 0:
        raise ValueError("horizon must be finite and positive")
    if not isinstance(records, list):
        raise ValueError("input must be a JSON list of event records")
    outcomes = Counter({name: 0 for name in OUTCOMES.values()})
    confusion = Counter({name: 0 for name in ("TP", "TN", "FP", "FN")})
    invalid = Counter()
    events = []
    seen = set()
    for record in records:
        event_id = record["event_id"]
        if not isinstance(event_id, str) or not event_id.strip() or event_id in seen:
            raise ValueError("event_id must be a non-empty unique string")
        seen.add(event_id)
        if not require_bool(record["valid_pair"]):
            reason = record["invalid_reason"]
            if not isinstance(reason, str) or not reason.strip():
                raise ValueError("invalid pairs require a non-empty invalid_reason")
            invalid[reason] += 1
            events.append({"event_id": event_id, "outcome": "INVALID_PAIR", "reason": reason})
            continue
        outcome = classify_pair(
            branch_violation(record["branch_e"], horizon_s),
            branch_violation(record["branch_f"], horizon_s),
        )
        label = classify_confusion(record["decision_intervene"], outcome)
        outcomes[outcome] += 1
        confusion[label] += 1
        events.append({"event_id": event_id, "outcome": outcome, "confusion": label})

    tp, fp, fn = confusion["TP"], confusion["FP"], confusion["FN"]
    return {
        "horizon_s": horizon_s,
        "total_pairs": len(records),
        "valid_pairs": sum(outcomes.values()),
        "invalid_pairs": sum(invalid.values()),
        "invalid_reasons": dict(invalid),
        "outcomes": dict(outcomes),
        "confusion": dict(confusion),
        "precision": tp / (tp + fp) if tp + fp else None,
        "recall": tp / (tp + fn) if tp + fn else None,
        "events": events,
    }
