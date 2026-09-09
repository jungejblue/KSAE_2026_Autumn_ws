"""Binary violations. Collision callbacks are joined by their original frame offline."""

import math


def tri_or(a, b):
    if a is True or b is True:
        return True
    if a is None or b is None:
        return None
    return False


def classify_wheels(points, corridor):
    if len(points) != 4 or not all(
        len(p) == 3 and all(math.isfinite(v) for v in p) for p in points
    ):
        return {"lane_outside_at_frame": None, "wheel_outside": None, "coverage_valid": False}
    outside = [not corridor.contains(p) for p in points]
    return {"lane_outside_at_frame": any(outside), "wheel_outside": outside, "coverage_valid": True}


def finalize_rows(rows, events, covered_from, covered_through):
    frames = {}
    for e in events:
        frames.setdefault(e["frame"], []).append(e["event_id"])
    seen = False
    for row in rows:
        row = dict(row)
        f = row["frame"]
        covered = covered_from <= f <= covered_through
        collision = True if f in frames else (False if covered else None)
        row["collision_event_ids"] = frames.get(f, [])
        row["collision_at_frame"] = collision
        row["collision_coverage_valid"] = covered
        row["coverage_valid"] = row["coverage_valid"] and covered
        row["violation_at_frame"] = tri_or(collision, row["lane_outside_at_frame"])
        seen = tri_or(seen, row["violation_at_frame"])
        row["violation_seen"] = seen
        yield row


def collision_episodes(events):
    """One episode per actor and adjacent event frames; never alters binary labels."""
    result, last = [], {}
    for event in sorted(events, key=lambda e: (e["frame"], e["event_id"])):
        key, frame = event["other_actor_id"], event["frame"]
        if key not in last or frame > last[key]["last_frame"] + 1:
            episode = {
                "other_actor_id": key,
                "first_frame": frame,
                "last_frame": frame,
                "events": 0,
            }
            result.append(episode)
            last[key] = episode
        last[key]["last_frame"] = frame
        last[key]["events"] += 1
    return result
