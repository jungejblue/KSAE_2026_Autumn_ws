"""Offline surrogate metrics. Constant velocity/orientation; no control prediction."""

import itertools
import math

from ksae_2026_autumn.violation_geometry import fixed_wheel_points, rotation

HORIZON_S = 3.0
STEP_S = 0.05


def vector(v, n=3):
    if len(v) != n or not all(math.isfinite(x) for x in v):
        raise ValueError("Missing/nonfinite geometry or motion")
    return v


def dot(a, b):
    return sum(x * y for x, y in zip(a, b, strict=True))


def hull(points):
    points = sorted(set(tuple(p) for p in points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def half(seq):
        out = []
        for p in seq:
            while len(out) > 1 and cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    result = half(points)[:-1] + half(reversed(points))[:-1]
    if len(result) < 3:
        raise ValueError("Degenerate projected bounding box")
    return result


def bbox(state):
    """XY convex projection and vertical interval of the full rotated 3-D bbox."""
    origin = vector(state["position_m"])
    R = rotation(vector(state["rpy_rad"]))
    B = rotation(vector(state["bbox_rpy_local_rad"]))
    center = vector(state["bbox_center_local_m"])
    extent = vector(state["bbox_extent_m"])
    if any(e <= 0 for e in extent):
        raise ValueError("Nonpositive bbox extent")
    corners = []
    for signs in itertools.product((-1, 1), repeat=3):
        local = [
            center[i] + sum(B[i][j] * extent[j] * signs[j] for j in range(3)) for i in range(3)
        ]
        corners.append([origin[i] + dot(R[i], local) for i in range(3)])
    return hull([p[:2] for p in corners]), (min(p[2] for p in corners), max(p[2] for p in corners))


def overlap_window(a, b, relative_speed):
    """Times when translated interval B overlaps stationary interval A."""
    if abs(relative_speed) < 1e-12:
        return (-math.inf, math.inf) if b[1] >= a[0] and b[0] <= a[1] else None
    x = (a[0] - b[1]) / relative_speed
    y = (a[1] - b[0]) / relative_speed
    return min(x, y), max(x, y)


def pair_ttc(ego, other, horizon=HORIZON_S):
    if not math.isfinite(horizon) or horizon <= 0:
        raise ValueError("Invalid horizon")
    a, az = bbox(ego)
    b, bz = bbox(other)
    va = vector(ego["velocity_mps"])
    vb = vector(other["velocity_mps"])
    dv = [y - x for x, y in zip(va, vb, strict=True)]
    enter, leave = 0.0, horizon
    intervals = [(az, bz, dv[2])]
    for poly in (a, b):
        for p, q in zip(poly, poly[1:] + poly[:1], strict=True):
            axis = [-(q[1] - p[1]), q[0] - p[0]]
            length = math.hypot(*axis)
            if length < 1e-12:
                continue
            axis = [v / length for v in axis]
            ap = [dot(p, axis) for p in a]
            bp = [dot(p, axis) for p in b]
            intervals.append(((min(ap), max(ap)), (min(bp), max(bp)), dot(dv[:2], axis)))
    for left, right, speed in intervals:
        window = overlap_window(left, right, speed)
        if window is None:
            return None
        enter = max(enter, window[0])
        leave = min(leave, window[1])
        if enter > leave + 1e-10:
            return None
    return max(0.0, min(horizon, enter))


def minimum_ttc(ego, actors, unmatched=(), horizon=HORIZON_S):
    # The recorded actor set contains vehicles/pedestrians, not static map objects.
    if unmatched:
        return {"ttc_s": None, "ttc_status": "unknown_actor_state", "ttc_actor_id": None}
    vector(ego["velocity_mps"])
    bbox(ego)
    best = None
    actor_id = None
    ids = set()
    for actor in actors:
        if actor["id"] in ids or actor["id"] == ego["id"]:
            raise ValueError("Duplicate/ego actor in TTC set")
        ids.add(actor["id"])
        if not actor["type_id"].startswith(("vehicle.", "walker.pedestrian.")):
            raise ValueError("Unexpected actor class in TTC set")
        value = pair_ttc(ego, actor, horizon)
        if value is not None and (best is None or value < best):
            best, actor_id = value, actor["id"]
    return {
        "ttc_s": best,
        "ttc_status": "predicted_overlap" if best is not None else "no_overlap_within_horizon",
        "ttc_actor_id": actor_id,
    }


def beyond_endpoint(points, corridor):
    for index, sign in [(0, -1), (-1, 1)]:
        sample = corridor.samples[index]
        angle = math.radians(sample["yaw_deg"])
        u = [math.cos(angle), math.sin(angle)]
        if any(sign * dot([p[i] - sample["xyz"][i] for i in range(2)], u) > 1e-6 for p in points):
            return True
    return False


def ttlc(state, corridor, horizon=HORIZON_S, step=STEP_S):
    """First sampled corridor exit under constant global velocity and orientation."""
    if step <= 0 or horizon <= 0 or not math.isfinite(step + horizon):
        raise ValueError("Invalid prediction grid")
    velocity = vector(state["velocity_mps"])
    wheels = fixed_wheel_points(state)
    if not all(corridor.contains(p) for p in wheels):
        return {"ttlc_s": 0.0, "ttlc_status": "already_outside", "ttlc_resolution_s": step}
    n = math.ceil(horizon / step)
    for k in range(1, n + 1):
        t = min(k * step, horizon)
        points = [[p[j] + velocity[j] * t for j in range(3)] for p in wheels]
        if not all(corridor.contains(p) for p in points):
            if beyond_endpoint(points, corridor):
                return {
                    "ttlc_s": None,
                    "ttlc_status": "route_endpoint_censored",
                    "ttlc_resolution_s": step,
                }
            return {"ttlc_s": t, "ttlc_status": "predicted_exit", "ttlc_resolution_s": step}
    return {"ttlc_s": None, "ttlc_status": "no_exit_on_prediction_grid", "ttlc_resolution_s": step}


def collision_speeds(events, ticks):
    """Event frame snapshot may be post-impact; use previous frame as an explicit proxy."""
    by_frame = {r["frame"]: r for r in ticks}
    out = []
    last = {}
    episode = -1
    for event in sorted(events, key=lambda e: (e["frame"], e["event_id"])):
        f = event["frame"]
        actor = event["other_actor_id"]
        same = actor in last and f <= last[actor][0] + 1
        if same:
            episode_id = last[actor][1]
        else:
            episode += 1
            episode_id = episode
        last[actor] = (f, episode_id)
        current = by_frame.get(f)
        previous = by_frame.get(f - 1)
        valid = bool(
            current
            and previous
            and abs(current["sim_time_s"] - previous["sim_time_s"] - STEP_S) < 1e-5
        )

        def speed(row):
            return (
                math.sqrt(sum(v * v for v in vector(row["ego"]["velocity_mps"]))) if row else None
            )

        pre = speed(previous) if valid else None
        at = speed(current)
        out.append(
            {
                "event_id": event["event_id"],
                "frame": f,
                "other_actor_id": actor,
                "other_actor_type": event.get("other_actor_type"),
                "episode_id": episode_id,
                "episode_start": not same,
                "collision_speed_proxy_mps": pre,
                "collision_speed_proxy_kmh": None if pre is None else pre * 3.6,
                "speed_sample_frame": f - 1 if valid else None,
                "sample_lead_s": STEP_S if valid else None,
                "event_frame_speed_mps": at,
                "status": "previous_frame_proxy" if valid else "missing_previous_frame",
            }
        )
    return out
