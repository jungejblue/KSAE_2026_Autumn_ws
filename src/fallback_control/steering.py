"""MKZ Ackermann steering conversion, based on physical front-wheel angles.

The normalized CARLA request scales the INNER front wheel angle. The predictive
bicycle model uses the equivalent axle angle. They differ even without lag.
All public angles here are RH left-positive radians; normalized input is CARLA
right-positive. Fixed vehicle-local wheel hubs supply track width, not the body width.
"""

import math


def front_track(geometry):
    if len(geometry.wheels) != 4:
        raise ValueError("fixed_four_wheel_profile_required_for_steering")
    front = sorted(geometry.wheels, key=lambda p: p[0])[-2:]
    track = abs(front[0][1] - front[1][1])
    if not math.isfinite(track) or not 0.8 < track < 2.5:
        raise ValueError("invalid_front_track")
    return track


def equivalent_from_inner(inner, wheelbase, track):
    """Signed inner angle -> bicycle angle: cot(eq) = cot(inner) + track/(2L)."""
    a = abs(inner)
    angle = math.atan2(
        wheelbase * math.sin(a),
        wheelbase * math.cos(a) + track / 2 * math.sin(a),
    )
    return math.copysign(angle, inner)


def normalized_from_equivalent(equivalent, inner_bound, wheelbase, track):
    if not math.isfinite(equivalent) or not 0 < inner_bound < math.pi / 2:
        raise ValueError("invalid_steering_request_or_bound")
    limit = equivalent_from_inner(inner_bound, wheelbase, track)
    a = min(abs(equivalent), limit)
    inner = math.atan2(
        wheelbase * math.sin(a),
        wheelbase * math.cos(a) - track / 2 * math.sin(a),
    )
    return max(-1.0, min(1.0, -math.copysign(inner / inner_bound, equivalent)))
