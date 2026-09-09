"""Deterministic in-corridor quintic candidates and acceleration-limited speed plan."""

import numpy as np

from .recovery import recovery_envelope
from .types import Plan


def lateral_polynomial(s, s0, d0, slope0, target, length):
    z = np.clip((np.asarray(s) - s0) / length, 0.0, 1.0)
    c0, c1, c2 = d0, slope0 * length, 0.0
    c3, c4, c5 = np.linalg.solve(
        [[1.0, 1.0, 1.0], [3.0, 4.0, 5.0], [6.0, 12.0, 20.0]], [target - c0 - c1, -c1, 0.0]
    )
    d = c0 + c1 * z + c2 * z * z + c3 * z**3 + c4 * z**4 + c5 * z**5
    dp = (c1 + 2 * c2 * z + 3 * c3 * z * z + 4 * c4 * z**3 + 5 * c5 * z**4) / length
    return d, dp


class LocalPlanner:
    def __init__(self, route, geometry, config):
        self.route, self.g, self.c = route, geometry, config
        self.last_offset = 0.0
        self.recovery = (0.0, 0.0)

    def build(self, state, obstacles, speed_limit=None, stop_s=None):
        c, r, g = self.c, self.route, self.g
        s0, d0, e0, speed = state
        vmax = min(
            c.cruise_speed, c.max_speed, speed_limit if speed_limit is not None else c.max_speed
        )
        vmax = max(0.0, vmax)
        if self.recovery[0] > 0 and self.recovery[1] < c.recovery_deadline:
            vmax = min(vmax, c.recovery_speed_limit)
        end = min(r.length, s0 + max(40.0, speed * speed / (2 * c.brake_decel) + speed * 4 + 10))
        samples = np.linspace(s0, max(s0 + 1e-5, end), max(3, int((end - s0) / 0.3) + 1))
        _, _, _, curvature, _ = r.sample(samples)
        initial_violation = max(0.0, -float(np.min(r.wheel_margins(state, g))))
        slope = np.tan(e0) * (1 - curvature[0] * d0)
        length = max(c.transition_length, speed * 2.5, abs(d0) * 4.0)
        candidates = []
        for target in (0.0,) if self.recovery[0] > 0 else c.candidate_offsets:
            d, dp = lateral_polynomial(samples, s0, d0, slope, target, length)
            e = np.arctan2(dp, np.maximum(0.25, 1 - curvature * d))
            xy = r.world(samples, d)
            _, _, cyaw, _, _ = r.sample(samples)
            psi = cyaw + e
            xs, disc_radius = g.discs
            hard_stop = min(
                r.length - g.front - c.stop_buffer,
                float(stop_s) if stop_s is not None else r.length,
            )
            for unsupported in r.unsupported_s:
                if unsupported >= s0 - 1:
                    hard_stop = min(hard_stop, unsupported - g.front - c.stop_buffer)
            reason = "route_end" if hard_stop < end else "cruise"
            trial = np.column_stack([samples, d, e, np.full_like(samples, speed)])
            lane = np.min(r.wheel_margins_batch(trial, g), axis=1)
            allowed = recovery_envelope(
                (samples - s0) / max(speed, vmax, 1.0) + self.recovery[1], self.recovery[0], c
            )
            # Only a predicted wheel-boundary conflict creates a lane stop.
            # The extra body margin is a soft preference evaluated by the MPC.
            invalid = np.flatnonzero((samples > s0 + 1e-6) & (lane < -allowed - c.feasibility_tol))
            if len(invalid):
                limit = max(s0, samples[invalid[0]] - c.stop_buffer)
                if limit < hard_stop:
                    hard_stop, reason = limit, "corridor_blocked"
            times = (samples - s0) / max(speed, vmax, 0.5)
            cx = xy[:, 0, None] + xs * np.cos(psi[:, None]) - g.box_y * np.sin(psi[:, None])
            cy = xy[:, 1, None] + xs * np.sin(psi[:, None]) + g.box_y * np.cos(psi[:, None])
            for obs in obstacles:
                dx = cx - (obs.x + obs.vx * times[:, None])
                dy = cy - (obs.y + obs.vy * times[:, None])
                clearance = np.min(np.hypot(dx, dy), axis=1)
                radii = disc_radius + obs.radius + c.obstacle_margin + c.prediction_growth * times
                blocked = np.flatnonzero(clearance < radii)
                if len(blocked):
                    first = int(blocked[0])
                    contact = samples[first]
                    if first:
                        # Interpolate the signed clearance across its first zero.
                        # Interpolation avoids ~0.3 m stop-station jumps between cells
                        # as the ego-relative sampling grid advances.
                        # A 1 cm upstream pad is conservative; the MPC still checks
                        # the full geometric obstacle constraints independently.
                        before = clearance[first - 1] - radii[first - 1]
                        after = clearance[first] - radii[first]
                        fraction = before / (before - after)
                        contact = samples[first - 1] + fraction * (
                            samples[first] - samples[first - 1]
                        )
                    limit = contact - c.stop_buffer - 0.01
                    if limit < hard_stop:
                        hard_stop, reason = limit, "obstacle_stop"
            heading = np.unwrap(psi)
            path_k = np.gradient(heading, samples)
            curve_cap = np.sqrt(c.lateral_accel / np.maximum(np.abs(path_k), 1e-3))
            remaining = np.maximum(0.0, hard_stop - samples)
            # Keep a fixed terminal margin and taper to zero before the hard
            # stop. Reaction distance belongs in speed planning, not a stop
            # station that advances every time the ego slows down.
            available = np.maximum(
                0.0, remaining - 0.5 - c.terminal_coast_reserve - speed * c.reaction_time
            )
            stop_cap = np.minimum(np.sqrt(2 * c.stop_plan_decel * available), available)
            cap = np.minimum(vmax, np.minimum(curve_cap, stop_cap))
            # Propagate downstream speed caps backward so a bend/stop is anticipated.
            for j in range(len(cap) - 2, -1, -1):
                cap[j] = min(
                    cap[j],
                    np.sqrt(cap[j + 1] ** 2 + 2 * c.brake_decel * (samples[j + 1] - samples[j])),
                )
            # Reference speed is a target; measured overspeed remains feasible in MPC.
            ss, vv = [s0], [speed]
            for k in range(c.horizon):
                dt = c.control_dt if k == 0 else c.dt
                desired = float(np.interp(ss[-1], samples, cap))
                vnext = max(
                    0.0,
                    vv[-1] + dt * np.clip(1.5 * (desired - vv[-1]), -c.brake_decel, c.accel_max),
                )
                sdot_factor = max(0.2, np.cos(float(np.interp(ss[-1], samples, e))))
                snext = min(r.length, ss[-1] + 0.5 * (vv[-1] + vnext) * dt * sdot_factor)
                ss.append(snext)
                vv.append(vnext)
            ss, vv = np.asarray(ss), np.asarray(vv)
            dd, ddp = lateral_polynomial(ss, s0, d0, slope, target, length)
            _, _, _, kk, _ = r.sample(ss)
            ee = np.arctan2(ddp, np.maximum(0.25, 1 - kk * dd))
            ref = np.column_stack([ss, dd, ee, vv])
            progress = min(max(0.0, hard_stop - s0), max(15.0, vmax * c.times[-1] + 5))
            score = -3 * progress + target * target + 0.5 * (target - self.last_offset) ** 2
            candidates.append((score, target, ref, hard_stop, reason))
        _, offset, ref, stop, reason = min(candidates, key=lambda x: x[0])
        self.last_offset = float(offset)
        mode = "avoid_in_lane" if abs(offset) > 1e-6 else "lane_recovery"
        if stop < ref[-1, 0] + c.stop_buffer:
            mode = reason
        return Plan(
            ref,
            mode,
            float(offset),
            float(stop),
            {
                "candidate_count": len(candidates),
                "initial_lane_violation_m": initial_violation,
                "recovery_initial_allowance_m": self.recovery[0],
                "recovery_elapsed_s": self.recovery[1],
                "stop_reason": reason,
                "stop_intent": reason != "cruise" and stop - s0 < 1.5 + c.terminal_coast_reserve,
            },
        )
