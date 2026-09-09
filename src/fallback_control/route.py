"""Route-ordered Frenet geometry. No nearest-lane reassignment after departure."""

import numpy as np


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Route:
    def __init__(self, xy, widths, spacing=0.5, unsupported=None):
        xy, widths = np.asarray(xy, float), np.asarray(widths, float)
        if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3:
            raise ValueError("Need at least three route XY points")
        if widths.shape != (len(xy),) or not np.isfinite(xy).all() or not np.isfinite(widths).all():
            raise ValueError("Invalid route data")
        if np.min(widths) <= 0 or spacing <= 0:
            raise ValueError("Invalid lane width/spacing")
        keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-4]
        xy, widths = xy[keep], widths[keep]
        if len(xy) < 3:
            raise ValueError("Degenerate route")
        s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        self.s = np.linspace(0, s[-1], max(3, int(np.ceil(s[-1] / spacing)) + 1))
        self.xy = np.column_stack([np.interp(self.s, s, xy[:, j]) for j in (0, 1)])
        self.width = np.interp(self.s, s, widths)
        tangent = np.gradient(self.xy, self.s, axis=0)
        self.yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        self.kappa = np.gradient(self.yaw, self.s)
        self.length = float(self.s[-1])
        self.unsupported_s = [] if unsupported is None else list(unsupported)
        self._progress = None

    def sample(self, s):
        a = np.asarray(s)
        return (
            np.interp(a, self.s, self.xy[:, 0]),
            np.interp(a, self.s, self.xy[:, 1]),
            np.interp(a, self.s, self.yaw),
            np.interp(a, self.s, self.kappa),
            np.interp(a, self.s, self.width),
        )

    def world(self, s, d):
        x, y, yaw, _, _ = self.sample(s)
        return np.stack([x - d * np.sin(yaw), y + d * np.cos(yaw)], axis=-1)

    def project(self, ego, max_distance=6.0):
        # Bounded forward search prevents jumping to a later lap or junction branch.
        lo, hi = (
            (0, min(self.length, 30.0))
            if self._progress is None
            else (
                max(0.0, self._progress - 3.0),
                min(self.length, self._progress + 12.0 + ego.speed * 0.2),
            )
        )
        mask = (self.s[:-1] >= lo - 0.6) & (self.s[:-1] <= hi)
        ids = np.flatnonzero(mask)
        a, b = self.xy[ids], self.xy[ids + 1]
        ab = b - a
        t = np.clip(
            np.sum((np.array([ego.x, ego.y]) - a) * ab, axis=1) / np.sum(ab * ab, axis=1), 0, 1
        )
        q = a + t[:, None] * ab
        err = np.linalg.norm(q - [ego.x, ego.y], axis=1)
        headings = np.arctan2(ab[:, 1], ab[:, 0])
        # Geometry defines s. Heading only rejects incompatible route branches;
        # adding heading to distance can choose a distant endpoint and lose position.
        aligned = np.abs(wrap(ego.yaw - headings)) <= 1.2
        score = np.where(aligned, err, np.inf)
        j = int(np.argmin(score))
        if err[j] > max_distance or abs(wrap(ego.yaw - headings[j])) > 1.2:
            raise ValueError("route_projection_invalid")
        s = self.s[ids[j]] + t[j] * (self.s[ids[j] + 1] - self.s[ids[j]])
        _, _, yaw, _, _ = self.sample(s)
        d = float(np.dot(np.array([ego.x, ego.y]) - q[j], [-np.sin(yaw), np.cos(yaw)]))
        old_progress = self._progress
        self._progress = float(s)
        self.last_projection = {
            "projection_roundtrip_error_m": float(
                np.linalg.norm(self.world(s, d) - np.array([ego.x, ego.y]))
            ),
            "projection_station_step_m": None if old_progress is None else float(s - old_progress),
        }
        return np.array([s, d, float(wrap(ego.yaw - yaw)), ego.speed])

    def lane_margins(self, state, geometry, margin=0.0):
        return self.lane_margins_batch(np.asarray(state)[None, :], geometry, margin)[0]

    def wheel_margins(self, state, geometry):
        return self.wheel_margins_batch(np.asarray(state)[None, :], geometry)[0]

    def wheel_margins_batch(self, states, geometry):
        # Fixed actor-local wheel hubs. A geometry without wheel hubs
        # (generic non-CARLA unit tests) falls back to the conservative body box.
        points = np.asarray(geometry.wheels) if geometry.wheels else geometry.body_points
        return self._point_margins_batch(states, points, 0.0)

    def lane_margins_batch(self, states, geometry, margin=0.0):
        return self._point_margins_batch(states, geometry.body_points, margin)

    def _point_margins_batch(self, states, points, margin):
        states = np.asarray(states)
        s, d, e = states[:, 0], states[:, 1], states[:, 2]
        _, _, yaw, _, _ = self.sample(s)
        center = self.world(s, d)
        psi = yaw + e
        px, py = points.T
        wx = center[:, 0, None] + np.cos(psi[:, None]) * px - np.sin(psi[:, None]) * py
        wy = center[:, 1, None] + np.sin(psi[:, None]) * px + np.cos(psi[:, None]) * py
        station = np.clip(
            s[:, None] + px * np.cos(e[:, None]) - py * np.sin(e[:, None]), 0, self.length
        )
        # Refine each body's nearest station within its local route neighbourhood.
        # The first-order s+longitudinal-offset formula alone is biased in bends.
        seed = station.copy()
        for _ in range(3):
            cx, cy, cyaw, curvature, _ = self.sample(station)
            longitudinal = (wx - cx) * np.cos(cyaw) + (wy - cy) * np.sin(cyaw)
            lateral = -(wx - cx) * np.sin(cyaw) + (wy - cy) * np.cos(cyaw)
            step = longitudinal / np.maximum(0.3, 1 - curvature * lateral)
            station = np.clip(
                station + np.clip(step, -1.0, 1.0),
                np.maximum(0.0, seed - 3.0),
                np.minimum(self.length, seed + 3.0),
            )
        cx, cy, cyaw, _, widths = self.sample(station)
        lat = -(wx - cx) * np.sin(cyaw) + (wy - cy) * np.cos(cyaw)
        return np.concatenate([widths / 2 - margin - lat, widths / 2 - margin + lat], axis=1)

    def reset(self):
        self._progress = None
