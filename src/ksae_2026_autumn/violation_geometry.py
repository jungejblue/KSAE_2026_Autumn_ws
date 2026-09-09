"""Frozen vehicle-local reference points, independent of UE world origin."""

import copy
import math

PROFILE = {
    "local_m": [
        [1.4718363032231172, -0.796469804207531, 0.3679311748594045],
        [1.4718409852589618, 0.7964601707774526, 0.36793113671243194],
        [-1.3886329631317518, -0.7964615785460475, 0.36793123207986356],
        [-1.3886478123423094, 0.7964684082307052, 0.36793119393289087],
    ],
    "max_coordinate_range_m": 0.024798928350210192,
    "frames": [14557, 14558, 14559, 14560, 14561, 14562, 14563, 14564, 14565, 14566],
    "id": "mkz2020_fixed_hubs_v1",
    "vehicle_type": "vehicle.lincoln.mkz_2020",
    "method": "median_local_coordinates_of_10_stationary_Town01_frames",
    "source_sha256": "d41d5da1568e8495a7e836348604360a8184566da6fd32aefaadb91be777be8f",
    "definition": (
        "Fixed wheel hub reference points; not live tyre contact patches. "
        "z is nominal hub height for road-layer selection."
    ),
}


def rotation(rpy):
    if len(rpy) != 3 or not all(math.isfinite(v) for v in rpy):
        raise ValueError("Invalid ego orientation")
    r, p, y = rpy
    sr, cr, sp, cp, sy, cy = (
        math.sin(r),
        math.cos(r),
        math.sin(p),
        math.cos(p),
        math.sin(y),
        math.cos(y),
    )
    return [
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
        [sp, -cp * sr, cp * cr],
    ]


def profile_for(vehicle_type):
    if vehicle_type != PROFILE["vehicle_type"]:
        raise ValueError("Fixed wheel profile only supports vehicle.lincoln.mkz_2020")
    return copy.deepcopy(PROFILE)


def fixed_wheel_points(state):
    profile = profile_for(state["type_id"])
    origin = state["position_m"]
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError("Invalid ego position")
    matrix = rotation(state["rpy_rad"])
    return [
        [origin[i] + sum(matrix[i][j] * p[j] for j in range(3)) for i in range(3)]
        for p in profile["local_m"]
    ]


def validate_bbox(ego):
    profile = profile_for(ego.type_id)
    e, c = ego.bounding_box.extent, ego.bounding_box.location
    # Keep the known model guard; do not turn coordinate failures into extra lane tolerance.
    if abs(e.x - 2.44619) > 0.05 or abs(e.y - 0.91836) > 0.05:
        raise ValueError("Vehicle bounding box differs from the calibrated MKZ profile")
    if any(
        abs(p[0] - c.x) > e.x + 0.01 or abs(p[1] - c.y) > e.y + 0.01 for p in profile["local_m"]
    ):
        raise ValueError("Fixed wheel profile lies outside vehicle bounding box")
    return profile
