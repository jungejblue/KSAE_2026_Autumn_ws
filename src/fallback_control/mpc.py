"""Frenet NMPC with hard moving-obstacle DCBFs and relaxed path-error CLF.

The CLF is a candidate with slack, not a proof of closed-loop stability.
"""

import time

import casadi as ca
import numpy as np

from .types import Command


class MPC:
    def __init__(self, route, geometry, config):
        self.r, self.g, self.c = route, geometry, config
        self._warm = None
        self._build()

    def _build(self):
        r, g, c = self.r, self.g, self.c
        N, M = c.horizon, c.max_obstacles
        op = ca.Opti()
        self.op = op
        X, U, S = op.variable(4, N + 1), op.variable(2, N), op.variable(N)
        x0, ref = op.parameter(4), op.parameter(4, N + 1)
        obs = op.parameter(6, M)  # x,y,vx,vy,radius,active
        prev, lane_allow, steer_bound, stop_bound = (op.parameter() for _ in range(4))
        self.X, self.U, self.S = X, U, S
        self.params = (x0, ref, obs, prev, lane_allow, steer_bound, stop_bound)
        interps = [
            ca.interpolant(f"route_{name}", "linear", [r.s.tolist()], values.tolist())
            for name, values in [
                ("x", r.xy[:, 0]),
                ("y", r.xy[:, 1]),
                ("yaw", r.yaw),
                ("k", r.kappa),
                ("w", r.width),
            ]
        ]

        def sample(s):
            sc = ca.fmin(ca.fmax(s, 0), r.length)
            return [f(sc) for f in interps]

        def world(z):
            rx, ry, yaw, _, _ = sample(z[0])
            return rx - z[1] * ca.sin(yaw), ry + z[1] * ca.cos(yaw), yaw + z[2]

        sz, su, st, so = ca.MX.sym("z", 4), ca.MX.sym("u", 2), ca.MX.sym("t"), ca.MX.sym("ob", 6)
        opts = {"never_inline": True, "cse": True}
        world = ca.Function("world_pose", [sz], list(world(sz))).expand("world_pose", opts)

        def dynamics(z, u):
            kappa = sample(z[0])[3]
            # Domain constrained separately; epsilon is numerical guard only.
            denom = ca.fmax(1 - kappa * z[1], 0.15)
            ds = z[3] * ca.cos(z[2]) / denom
            return ca.vertcat(
                ds, z[3] * ca.sin(z[2]), z[3] / g.wheelbase * ca.tan(u[1]) - kappa * ds, u[0]
            )

        dynamics = ca.Function("rhs", [sz, su], [dynamics(sz, su)]).expand("rhs", opts)

        def rk4(z, u, dt):
            k1 = dynamics(z, u)
            k2 = dynamics(z + dt * k1 / 2, u)
            k3 = dynamics(z + dt * k2 / 2, u)
            k4 = dynamics(z + dt * k3, u)
            return z + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6

        rk4 = ca.Function("rk4_step", [sz, su, st], [rk4(sz, su, st)], opts)

        def lane(z):
            wx, wy, psi = world(z)
            hs = []
            for px, py in g.body_points:
                cx = wx + px * ca.cos(psi) - py * ca.sin(psi)
                cy = wy + px * ca.sin(psi) + py * ca.cos(psi)
                st = z[0] + px * ca.cos(z[2]) - py * ca.sin(z[2])
                rx, ry, yaw, _, width = sample(st)
                d = -(cx - rx) * ca.sin(yaw) + (cy - ry) * ca.cos(yaw)
                hs += [width / 2 - c.lane_margin - d, width / 2 - c.lane_margin + d]
            return ca.vertcat(*hs)

        lane = ca.Function("lane_margins", [sz], [lane(sz)]).expand("lane_margins", opts)

        xs, rad = g.discs

        def barriers(z, t, obstacle):
            wx, wy, psi = world(z)
            ox, oy = obstacle[0] + obstacle[2] * t, obstacle[1] + obstacle[3] * t
            R = rad + obstacle[4] + c.obstacle_margin + c.prediction_growth * t
            hs = []
            for px in xs:
                cx = wx + px * ca.cos(psi) - g.box_y * ca.sin(psi)
                cy = wy + px * ca.sin(psi) + g.box_y * ca.cos(psi)
                # Scale squared-distance constraints to reduce poor conditioning.
                h = ((cx - ox) ** 2 + (cy - oy) ** 2 - R * R) / (R * R)
                hs.append(obstacle[5] * h + (1 - obstacle[5]))
            return ca.vertcat(*hs)

        barriers = ca.Function("barriers", [sz, st, so], [barriers(sz, st, so)]).expand(
            "barriers", opts
        )

        def V(z, rr):
            e = ca.atan2(ca.sin(z[2] - rr[2]), ca.cos(z[2] - rr[2]))
            return (z[1] - rr[1]) ** 2 + 3 * e**2

        op.subject_to(X[:, 0] == x0)
        op.subject_to(S >= 0)
        op.subject_to(op.bounded(0, X[0, :], r.length))
        op.subject_to(X[0, :] <= ca.fmax(x0[0], stop_bound))
        op.subject_to(op.bounded(-1.2, X[2, :], 1.2))
        # Existing overspeed is allowed and penalized, never silently clipped.
        op.subject_to(op.bounded(0, X[3, :], ca.fmax(c.max_speed, x0[3])))
        op.subject_to(op.bounded(-c.brake_decel, U[0, :], c.accel_max))
        op.subject_to(op.bounded(-steer_bound, U[1, :], steer_bound))
        objective = 0
        for k in range(N + 1):
            z = X[:, k]
            t = float(c.times[k])
            op.subject_to(1 - sample(z[0])[3] * z[1] >= 0.25)
            allowed = lane_allow * np.exp(-c.recovery_rate * max(0.0, t - c.recovery_grace))
            op.subject_to(lane(z) >= -allowed)
            # Every knot must remain outside every inflated obstacle. No CBF slack.
            for j in range(M):
                op.subject_to(barriers(z, t, obs[:, j]) >= 0)
            objective += (
                0.4 * (z[0] - ref[0, k]) ** 2 + 12 * V(z, ref[:, k]) + 4 * (z[3] - ref[3, k]) ** 2
            )
            if k == N:
                continue
            dt = c.control_dt if k == 0 else c.dt
            nxt, u = X[:, k + 1], U[:, k]
            op.subject_to(nxt == rk4(z, u, dt))
            last = prev if k == 0 else U[1, k - 1]
            op.subject_to(op.bounded(-c.steer_rate * dt, u[1] - last, c.steer_rate * dt))
            op.subject_to(
                op.bounded(
                    -c.lateral_accel, z[3] ** 2 / g.wheelbase * ca.tan(u[1]), c.lateral_accel
                )
            )
            op.subject_to(V(nxt, ref[:, k + 1]) <= (1 - c.clf_rate * dt) * V(z, ref[:, k]) + S[k])
            objective += (
                c.clf_weight * S[k] ** 2
                + 0.1 * u[0] ** 2
                + 0.3 * u[1] ** 2
                + 5 * (u[1] - last) ** 2
            )
            gamma = 1 - (1 - c.cbf_gamma) ** (dt / c.dt)
            mid = rk4(z, u, dt / 2)
            for j in range(M):
                h0 = barriers(z, t, obs[:, j])
                h1 = barriers(nxt, t + dt, obs[:, j])
                op.subject_to(h1 - (1 - gamma) * h0 >= 0)
                # One midpoint check supplements knot constraints; not a continuous-time proof.
                op.subject_to(barriers(mid, t + dt / 2, obs[:, j]) >= 0)
            op.subject_to(
                lane(rk4(z, u, dt / 2))
                >= -lane_allow * np.exp(-c.recovery_rate * max(0.0, t + dt / 2 - c.recovery_grace))
            )
        op.minimize(objective)
        self._check = ca.Function("check", [op.x, op.p], [op.g, op.lbg, op.ubg])
        op.solver(
            "ipopt",
            {
                "expand": False,
                "print_time": False,
                "jit": c.native,
                "compiler": "shell",
                "jit_options": {"flags": ["-O1"]},
            },
            {
                "print_level": 0,
                "sb": "yes",
                "max_iter": c.max_iter,
                "tol": 1e-5,
                "constr_viol_tol": c.feasibility_tol,
                "max_cpu_time": c.solver_budget_s,
            },
        )

    def solve(self, state, plan, obstacles, steer_bound=None):
        c, op = self.c, self.op
        start = time.perf_counter()
        state = np.asarray(state, float)
        if len(obstacles) > c.max_obstacles:
            return Command(diagnostics={"reason": "obstacle_capacity_exceeded"})
        p_obs = np.zeros((6, c.max_obstacles))
        p_obs[4, :] = 1.0
        for j, ob in enumerate(obstacles):
            p_obs[:, j] = [ob.x, ob.y, ob.vx, ob.vy, ob.radius, 1.0]
        allow = max(0.0, -float(np.min(self.r.lane_margins(state, self.g, c.lane_margin))))
        bound = min(self.g.max_steer, self.g.max_steer if steer_bound is None else steer_bound)
        values = (state, plan.reference.T, p_obs, self.applied_steer, allow, bound, plan.stop_s)
        for parameter, value in zip(self.params, values, strict=False):
            op.set_value(parameter, value)
        if self._warm is None:
            op.set_initial(self.X, plan.reference.T)
            op.set_initial(self.U, np.zeros((2, c.horizon)))
            op.set_initial(self.S, 0.1)
        else:
            xp, up, sp = self._warm
            op.set_initial(self.X, np.column_stack([state, xp[:, 2:], xp[:, -1]]))
            op.set_initial(self.U, np.column_stack([up[:, 1:], up[:, -1]]))
            op.set_initial(self.S, np.r_[sp[1:], sp[-1]])
        try:
            sol = op.solve()
            x, u, slack = sol.value(self.X), sol.value(self.U), sol.value(self.S)
            elapsed = time.perf_counter() - start
            gg, lb, ub = [
                np.asarray(a).ravel() for a in self._check(sol.value(op.x), op.value(op.p))
            ]
            residual = float(max(0.0, np.max(lb - gg), np.max(gg - ub)))
            diag = {
                "reason": "mpc",
                "solve_s": elapsed,
                "constraint_residual": residual,
                "clf_slack_max": float(np.max(slack)),
                "lane_recovery_allowance_m": allow,
                "iterations": op.stats().get("iter_count"),
                "solver_status": self._status(),
                "plan_mode": plan.mode,
                "selected_offset_m": plan.offset,
                "stop_s": plan.stop_s,
                "obstacle_count": len(obstacles),
            }
            if (
                not all(np.isfinite(a).all() for a in (x, u, slack, gg))
                or residual > c.feasibility_tol
            ):
                diag["reason"] = "invalid_solution"
                self._warm = None
                return Command(diagnostics=diag)
            if elapsed > c.solver_budget_s:
                diag["reason"] = "solver_deadline_missed"
                self._warm = None
                return Command(diagnostics=diag)
            self._warm = (x, u, slack)
            return Command(float(u[0, 0]), float(u[1, 0]), float(x[3, 1]), False, diag, x.T)
        except RuntimeError as exc:
            self._warm = None
            return Command(
                diagnostics={
                    "reason": "solver_failed",
                    "solve_s": time.perf_counter() - start,
                    "solver_status": self._status(),
                    "error": str(exc).splitlines()[-1][:250],
                    "plan_mode": plan.mode,
                    "lane_recovery_allowance_m": allow,
                }
            )

    def _status(self):
        try:
            return self.op.stats().get("return_status", "")
        except RuntimeError:
            return "solver_not_initialized"

    applied_steer = 0.0

    def reset(self):
        self._warm = None
