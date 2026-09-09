"""Fixed-size sampled predictive controller with discrete CBF rejection.

Not an equivalent optimizer to IPOPT: checks a finite set of feedback-generated
control sequences. Wheel boundaries and obstacle CBF are checked at horizon knots/midpoints.
The additional body margin is a soft cost, not a wheel-violation tolerance.
CLF violation is penalized as explicit nonnegative slack, not a safety slack.
"""

import time

import numpy as np

from .planner import lateral_polynomial
from .recovery import recovery_envelope
from .types import Command


class SamplingMPC:
    def __init__(self, route, geometry, config):
        self.r, self.g, self.c = route, geometry, config
        # Deterministic 5 x 3 x 3 policies; no data-dependent search iterations.
        mesh = np.meshgrid(
            [-4.0, -2.0, 0.0, 1.0, 2.0], [0.65, 1.0, 1.4], [-1.0, 0.0, 1.0], indexing="ij"
        )
        self.policies = np.stack([v.ravel() for v in mesh], axis=1)
        self.applied_steer = 0.0
        self.previous_acceleration = 0.0
        self.coast_active = False

    def reset(self):
        self.previous_acceleration = 0.0
        self.coast_active = False

    def _rhs(self, z, u):
        kappa = np.interp(z[:, 0], self.r.s, self.r.kappa)
        ds = z[:, 3] * np.cos(z[:, 2]) / np.maximum(0.15, 1 - kappa * z[:, 1])
        return np.column_stack(
            [
                ds,
                z[:, 3] * np.sin(z[:, 2]),
                z[:, 3] / self.g.wheelbase * np.tan(u[:, 1]) - kappa * ds,
                u[:, 0],
            ]
        )

    def _rk4(self, z, u, dt):
        a = self._rhs(z, u)
        b = self._rhs(z + dt * a / 2, u)
        cc = self._rhs(z + dt * b / 2, u)
        d = self._rhs(z + dt * cc, u)
        return z + dt * (a + 2 * b + 2 * cc + d) / 6

    def _barriers(self, states, times, obstacles):
        """Output: policy x time x obstacle x ego disc. Empty obstacle axis allowed."""
        shape = states.shape[:2]
        flat = states.reshape(-1, 4)
        world = self.r.world(flat[:, 0], flat[:, 1]).reshape(*shape, 2)
        yaw = np.interp(flat[:, 0], self.r.s, self.r.yaw).reshape(shape) + states[:, :, 2]
        xs, radius = self.g.discs
        cx = (
            world[:, :, 0, None]
            + xs * np.cos(yaw[:, :, None])
            - self.g.box_y * np.sin(yaw[:, :, None])
        )
        cy = (
            world[:, :, 1, None]
            + xs * np.sin(yaw[:, :, None])
            + self.g.box_y * np.cos(yaw[:, :, None])
        )
        if not obstacles:
            return np.empty((*shape, 0, len(xs)))
        ob = np.array([[o.x, o.y, o.vx, o.vy, o.radius] for o in obstacles])
        ox = ob[None, :, 0] + times[:, None] * ob[None, :, 2]
        oy = ob[None, :, 1] + times[:, None] * ob[None, :, 3]
        radius_sum = (
            radius
            + ob[None, :, 4]
            + self.c.obstacle_margin
            + self.c.prediction_growth * times[:, None]
        )
        distance2 = (cx[:, :, None, :] - ox[None, :, :, None]) ** 2 + (
            cy[:, :, None, :] - oy[None, :, :, None]
        ) ** 2
        return distance2 / radius_sum[None, :, :, None] ** 2 - 1

    def solve(self, state, plan, obstacles, steer_bound=None):
        start = time.perf_counter()
        c, g, r = self.c, self.g, self.r
        N, P = c.horizon, len(self.policies)
        state = np.asarray(state, float)
        if len(obstacles) > c.max_obstacles:
            return Command(diagnostics={"reason": "obstacle_capacity_exceeded"})
        bound = min(g.max_steer, g.max_steer if steer_bound is None else steer_bound)
        times = c.times
        dts = np.diff(times)
        X = np.zeros((P, N + 1, 4))
        mid = np.zeros((P, N, 4))
        U = np.zeros((P, N, 2))
        coasting = np.full(P, self.coast_active, dtype=bool)
        first_coast = np.zeros(P, dtype=bool)
        X[:, 0, :] = state
        ref = plan.reference
        recovery_pacing = (
            plan.diagnostics.get("recovery_initial_allowance_m", 0.0) > 0
            and plan.diagnostics.get("recovery_elapsed_s", 0.0) < c.recovery_deadline
        )
        steer_rate = min(c.steer_rate, c.recovery_steer_rate) if recovery_pacing else c.steer_rate
        for k, dt in enumerate(dts):
            z = X[:, k]
            # Route preview creates turning candidates before the rear axle reaches
            # a bend. Vary lookahead over the existing deterministic policy grid.
            look = (3.0 + 0.4 * z[:, 3]) / self.policies[:, 1]
            center = r.world(z[:, 0], z[:, 1])
            target_s = np.minimum(r.length, z[:, 0] + look)
            start_k = np.interp(state[0], r.s, r.kappa)
            target_d, _ = lateral_polynomial(
                target_s,
                state[0],
                state[1],
                np.tan(state[2]) * (1 - start_k * state[1]),
                plan.offset,
                max(c.transition_length, state[3] * 2.5, abs(state[1]) * 4.0),
            )
            if plan.diagnostics.get("recovery_initial_allowance_m", 0.0) > 0:
                target_d = plan.offset
            target = r.world(target_s, target_d)
            displacement = target - center
            yaw = np.interp(z[:, 0], r.s, r.yaw) + z[:, 2]
            alpha = np.arctan2(displacement[:, 1], displacement[:, 0]) - yaw
            desired = np.arctan2(
                2 * g.wheelbase * np.sin(alpha),
                np.maximum(0.5, np.linalg.norm(displacement, axis=1)),
            )
            if k == 0:
                desired += self.policies[:, 2] * steer_rate * dt
            previous = self.applied_steer if k == 0 else U[:, k - 1, 1]
            delta = np.clip(
                desired,
                np.maximum(-bound, previous - steer_rate * dt),
                np.minimum(bound, previous + steer_rate * dt),
            )
            nominal = (ref[k + 1, 3] - z[:, 3]) / dt
            # Include the continuous nominal acceleration: the old 1/2 m/s^2
            # quantization made speed regulation chatter around the target.
            bias = np.where(self.policies[:, 0] == 0.0, 0.0, self.policies[:, 0] * 0.25)
            accel = np.clip(nominal + bias, -c.brake_decel, c.accel_max)
            normal_stop = plan.diagnostics.get("stop_reason", "cruise") != "cruise"
            if normal_stop:
                accel = np.maximum(accel, -c.stop_control_decel)
            if plan.diagnostics.get("stop_intent", False):
                accel = np.minimum(accel, -np.minimum(c.stop_control_decel, 0.6 * z[:, 3]))
            # A positive policy bias must not request acceleration above the
            # recovery speed cap. Keep the existing jerk bound on transitions.
            if recovery_pacing:
                accel = np.where(z[:, 3] >= c.recovery_speed_limit, np.minimum(accel, 0.0), accel)
            previous_a = self.previous_acceleration if k == 0 else U[:, k - 1, 0]
            accel = np.clip(
                accel, previous_a - c.normal_jerk_limit * dt, previous_a + c.normal_jerk_limit * dt
            )
            # Model zero-pedal deceleration for ALL normal low-speed slowing,
            # including a moving-obstacle plan with stop_intent=False. Latch the
            # mode through speed rebound; release on nonnegative desired input.
            normal = self.policies[:, 0] > -3.9
            coasting = normal & (accel < 0) & (
                coasting | (z[:, 3] < c.terminal_coast_speed)
            )
            accel = np.where(coasting, -c.terminal_coast_decel, accel)
            if k == 0:
                first_coast = coasting.copy()
            # Retain a prompt full-deceleration safety candidate, explicitly logged.
            accel = np.where(self.policies[:, 0] <= -3.9, -c.brake_decel, accel)
            if plan.diagnostics.get("stop_intent", False):
                accel = np.minimum(accel, 0.0)
            accel = np.maximum(accel, -z[:, 3] / dt)  # reach zero speed without reversing
            U[:, k] = np.column_stack([accel, delta])
            mid[:, k] = self._rk4(z, U[:, k], dt / 2)
            X[:, k + 1] = self._rk4(z, U[:, k], dt)
        rollout_s = time.perf_counter() - start
        tol = c.feasibility_tol
        allow = max(0.0, -float(np.min(r.wheel_margins(state, g))))
        flat = X.reshape(-1, 4)
        flat_mid = mid.reshape(-1, 4)
        lane = r.wheel_margins_batch(flat, g).reshape(P, N + 1, -1)
        lane_mid = r.wheel_margins_batch(flat_mid, g).reshape(P, N, -1)
        initial_allow = plan.diagnostics.get("recovery_initial_allowance_m", allow)
        recovery_age = plan.diagnostics.get("recovery_elapsed_s", 0.0)
        body_margin = r.lane_margins_batch(flat, g, c.lane_margin).reshape(P, N + 1, -1)
        body_deficit = np.maximum(0.0, -np.min(body_margin, axis=2))
        envelope = recovery_envelope(times + recovery_age, initial_allow, c)
        tm = times[:-1] + dts / 2
        envelope_mid = recovery_envelope(tm + recovery_age, initial_allow, c)
        h = self._barriers(X, times, obstacles)
        hm = self._barriers(mid, tm, obstacles)
        gamma = 1 - (1 - c.cbf_gamma) ** (dts / c.dt)
        cbf = h[:, 1:] - (1 - gamma[None, :, None, None]) * h[:, :-1]
        lane_ok = np.all(lane + envelope[None, :, None] >= -tol, axis=(1, 2)) & np.all(
            lane_mid + envelope_mid[None, :, None] >= -tol, axis=(1, 2)
        )
        obstacle_ok = (
            np.all(h >= -tol, axis=(1, 2, 3))
            & np.all(hm >= -tol, axis=(1, 2, 3))
            & np.all(cbf >= -tol, axis=(1, 2, 3))
        )
        domain_ok = np.ones(P, dtype=bool)
        for values in (X, mid):
            curvature = np.interp(values[:, :, 0], r.s, r.kappa)
            domain_ok &= np.all(
                (values[:, :, 0] >= -tol)
                & (values[:, :, 0] <= min(r.length, max(state[0], plan.stop_s)) + tol)
                & (np.abs(values[:, :, 2]) <= 1.2 + tol)
                & (values[:, :, 3] >= -tol)
                & (values[:, :, 3] <= max(c.max_speed, state[3]) + tol)
                & (1 - curvature * values[:, :, 1] >= 0.25 - tol),
                axis=1,
            )
        ay = X[:, :-1, 3] ** 2 / g.wheelbase * np.tan(U[:, :, 1])
        previous = np.column_stack([np.full(P, self.applied_steer), U[:, :-1, 1]])
        rate_ok = np.all(np.abs(U[:, :, 1] - previous) <= steer_rate * dts[None, :] + tol, axis=1)
        input_ok = (
            np.all(
                (np.abs(U[:, :, 1]) <= bound + tol)
                & (U[:, :, 0] >= -c.brake_decel - tol)
                & (U[:, :, 0] <= c.accel_max + tol)
                & (np.abs(ay) <= c.lateral_accel + tol),
                axis=1,
            )
            & rate_ok
        )
        finite_ok = np.isfinite(X).all(axis=(1, 2)) & np.isfinite(U).all(axis=(1, 2))
        valid = lane_ok & obstacle_ok & domain_ok & input_ok & finite_ok
        err_e = np.arctan2(np.sin(X[:, :, 2]), np.cos(X[:, :, 2]))
        V = (X[:, :, 1] - plan.offset) ** 2 + 3 * err_e**2
        slack = np.maximum(0.0, V[:, 1:] - (1 - c.clf_rate * dts[None, :]) * V[:, :-1])
        cost = np.sum(
            0.4 * (X[:, :, 0] - ref[None, :, 0]) ** 2
            + 12 * V
            + 4 * (X[:, :, 3] - ref[None, :, 3]) ** 2,
            axis=1,
        )
        cost += np.sum(
            c.clf_weight * slack**2
            + 0.1 * U[:, :, 0] ** 2
            + 0.3 * U[:, :, 1] ** 2
            + 5 * (U[:, :, 1] - previous) ** 2,
            axis=1,
        )
        cost += c.body_margin_weight * np.sum(body_deficit**2, axis=1)
        # Align the action applied now with the planner's first acceleration.
        # Otherwise a cheap zero first action can defer acceleration beyond every replan.
        reference_accel = float(
            np.clip((ref[1, 3] - state[3]) / dts[0], -c.brake_decel, c.accel_max)
        )
        cost += (U[:, 0, 0] - reference_accel) ** 2
        diagnostics = {
            "reference_first_acceleration_mps2": reference_accel,
            "backend": "sampled_predictive",
            "recovery_pacing_active": recovery_pacing,
            "effective_steer_rate_radps": steer_rate,
            "lane_constraint": "fixed_wheel_hubs_hard_body_margin_soft",
            "candidate_count": P,
            "valid_candidates": int(np.sum(valid)),
            "rollout_s": rollout_s,
            "lane_recovery_allowance_m": allow,
            "recovery_initial_allowance_m": initial_allow,
            "recovery_elapsed_s": recovery_age,
            "plan_mode": plan.mode,
            "selected_offset_m": plan.offset,
            "stop_s": plan.stop_s,
            "stop_intent": bool(plan.diagnostics.get("stop_intent", False)),
            "obstacle_count": len(obstacles),
            "rejected_lane": int(np.sum(~lane_ok)),
            "rejected_obstacles": int(np.sum(~obstacle_ok)),
            "rejected_domain": int(np.sum(~domain_ok)),
            "rejected_input": int(np.sum(~input_ok)),
        }
        if not np.any(valid):
            diagnostics.update(
                reason="no_safe_candidate",
                solve_s=time.perf_counter() - start,
                constraint_residual=None,
                emergency_trajectory_verified=False,
            )
            return Command(diagnostics=diagnostics)
        normal_valid = valid & (self.policies[:, 0] > -3.9)
        selection = valid
        # Full braking is a safety reserve in every plan mode. A tracking-cost
        # improvement alone must not select it when a normal candidate passes
        # every identical hard constraint. Retain full braking if none does.
        if np.any(normal_valid):
            selection = normal_valid
        diagnostics["valid_normal_candidates"] = int(np.sum(normal_valid))
        best = int(np.argmin(np.where(selection, cost, np.inf)))
        # Report negative safety margins independently from the cost used for selection.
        margins = [
            np.min(lane[best] + envelope[:, None]),
            np.min(lane_mid[best] + envelope_mid[:, None]),
        ]
        if obstacles:
            margins.extend([np.min(h[best]), np.min(hm[best]), np.min(cbf[best])])
        elapsed = time.perf_counter() - start
        diagnostics.update(
            reason="sampled_mpc",
            solve_s=elapsed,
            constraint_residual=float(max(0.0, -min(margins))),
            clf_slack_max=float(np.max(slack[best])),
            selected_policy=int(best),
            predicted_body_deficit_max_m=float(np.max(body_deficit[best])),
            prompt_braking_policy=bool(self.policies[best, 0] <= -3.9),
            coast_requested=bool(first_coast[best]),
            selected_acceleration_mps2=float(U[best, 0, 0]),
            predicted_next_speed_mps=float(X[best, 1, 3]),
            reference_next_speed_mps=float(ref[1, 3]),
            reference_terminal_speed_mps=float(ref[-1, 3]),
        )
        if elapsed > c.solver_budget_s:
            diagnostics["reason"] = "solver_deadline_missed"
            return Command(diagnostics=diagnostics)
        self.previous_acceleration = float(U[best, 0, 0])
        self.coast_active = bool(first_coast[best])
        return Command(
            float(U[best, 0, 0]),
            float(U[best, 0, 1]),
            float(X[best, 1, 3]),
            False,
            diagnostics,
            X[best],
            U[best],
        )
