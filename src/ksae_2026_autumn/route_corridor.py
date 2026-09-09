"""Route-specific lane strips, with height separation and inspectable geometry."""

import math


def point_in_polygon(point, polygon):
    """Boundary is inside; 1e-9 m is floating-point arithmetic tolerance only."""
    x, y = point[:2]
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1], strict=True):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-12:
            continue
        cross = (x - a[0]) * dy - (y - a[1]) * dx
        dot = (x - a[0]) * dx + (y - a[1]) * dy
        if abs(cross) <= 1e-9 * length and -1e-9 <= dot <= length * length + 1e-9:
            return True
        if (a[1] > y) != (b[1] > y):
            if x < a[0] + (y - a[1]) * dx / dy:
                inside = not inside
    return inside


def sample(wp):
    t = wp.transform
    return {
        "xyz": [t.location.x, t.location.y, t.location.z],
        "yaw_deg": t.rotation.yaw,
        "width_m": wp.lane_width,
        "lane": [wp.road_id, wp.section_id, wp.lane_id],
        "s": wp.s,
        "junction": wp.is_junction,
    }


def edges(p):
    x, y, z = p["xyz"]
    a = math.radians(p["yaw_deg"])
    nx, ny = -math.sin(a), math.cos(a)
    half = p["width_m"] / 2
    return [[x - nx * half, y - ny * half, z], [x + nx * half, y + ny * half, z]]


class Corridor:
    def __init__(self, samples, height_tolerance=1.5):
        if len(samples) < 2:
            raise ValueError("A corridor needs at least two lane samples")
        self.samples = samples
        self.height_tolerance = height_tolerance
        self.patches = []
        self.warnings = []
        for a, b in zip(samples, samples[1:], strict=False):
            distance = math.dist(a["xyz"], b["xyz"])
            if distance < 1e-5:
                continue
            if distance > 3.0:
                raise ValueError(f"Route corridor gap {distance:.2f} m; densify the route")
            if a["width_m"] <= 0 or b["width_m"] <= 0:
                raise ValueError("Invalid lane width")
            # Lateral lane changes must not be replaced by a diagonal road-width strip.
            same_road = a["lane"][:2] == b["lane"][:2]
            if same_road and a["lane"][2] != b["lane"][2]:
                raise ValueError(
                    "Lane-change transition requires an explicit corridor; unsupported"
                )
            al, ar = edges(a)
            bl, br = edges(b)
            poly = [al, ar, br, bl]
            self.patches.append({"polygon": poly, "z": (a["xyz"][2] + b["xyz"][2]) / 2})
        if not self.patches:
            raise ValueError("Empty corridor")
        # A spatial index avoids scanning the whole route for every wheel.
        self.cells = {}
        for i, p in enumerate(self.patches):
            xs, ys = zip(*[(v[0], v[1]) for v in p["polygon"]], strict=True)
            for gx in range(math.floor(min(xs) / 5), math.floor(max(xs) / 5) + 1):
                for gy in range(math.floor(min(ys) / 5), math.floor(max(ys) / 5) + 1):
                    self.cells.setdefault((gx, gy), []).append(i)

    def contains(self, point):
        for i in self.cells.get((math.floor(point[0] / 5), math.floor(point[1] / 5)), []):
            p = self.patches[i]
            if abs(point[2] - p["z"]) <= self.height_tolerance:
                if point_in_polygon(point, p["polygon"]):
                    return True
        return False

    def to_dict(self):
        return {
            "method": "union_of_route_lane_edge_strips",
            "samples": self.samples,
            "patches": self.patches,
            "height_tolerance_m": self.height_tolerance,
            "boundary": "inside; numerical tolerance 1e-9 m",
            "limitations": (
                "Finite map sampling; alignment depends on route/map geometry. No auto widening."
            ),
        }


def from_route(carla_map, route):
    """Use the evaluator's dense route, extending endpoints for wheel footprint."""
    waypoints = []
    for entry in route:
        value = entry[0] if isinstance(entry, (tuple, list)) else entry
        loc = value.location if hasattr(value, "location") else value
        wp = carla_map.get_waypoint(loc)
        if wp is None:
            raise ValueError("Route position has no map waypoint")
        if not waypoints or wp.transform.location.distance(waypoints[-1].transform.location) > 0.05:
            waypoints.append(wp)
    if len(waypoints) < 2:
        raise ValueError("No usable dense route")
    before, after = [], []
    wp = waypoints[0]
    for _ in range(8):
        options = wp.previous(1.0)
        if len(options) != 1:
            break
        wp = options[0]
        before.append(wp)
    wp = waypoints[-1]
    for _ in range(8):
        options = wp.next(1.0)
        if len(options) != 1:
            break
        wp = options[0]
        after.append(wp)
    return Corridor([sample(w) for w in list(reversed(before)) + waypoints + after])
