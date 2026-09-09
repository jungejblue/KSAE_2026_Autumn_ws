"""Fallback-only API: supervisor decides when to apply this command."""

import time

import numpy as np

from .planner import LocalPlanner
from .sampling_mpc import SamplingMPC
from .types import Command


class FallbackController:
    def __init__(self, route, geometry, config):
        self.route, self.geometry, self.config = route, geometry, config
        self.planner = LocalPlanner(route, geometry, config)
        self._recovery_initial = None
        self._recovery_ticks = 0
        if config.backend == "ipopt":
            from .mpc import MPC

            self.mpc = MPC(route, geometry, config)
        else:
            self.mpc = SamplingMPC(route, geometry, config)

    def reset(self):
        self.route.reset()
        self.mpc.reset()
        self.planner.last_offset = 0.0
        self._recovery_initial = None
        self._recovery_ticks = 0

    def step(self, ego, obstacles, speed_limit=None, stop_s=None, steer_bound=None):
        start = time.perf_counter()
        arr = [ego.x, ego.y, ego.yaw, ego.speed, ego.steer]
        if not np.isfinite(arr).all() or ego.speed < -0.05:
            return Command(diagnostics={"reason": "invalid_ego_state"})
        for ob in obstacles:
            if not np.isfinite([ob.x, ob.y, ob.vx, ob.vy, ob.radius]).all() or ob.radius <= 0:
                return Command(diagnostics={"reason": "invalid_obstacle"})
        if speed_limit is not None and (not np.isfinite(speed_limit) or speed_limit < 0):
            return Command(diagnostics={"reason": "invalid_speed_limit"})
        if stop_s is not None and not np.isfinite(stop_s):
            return Command(diagnostics={"reason": "invalid_stop_station"})
        if steer_bound is not None and (not np.isfinite(steer_bound) or steer_bound <= 0):
            return Command(diagnostics={"reason": "invalid_steer_bound"})
        if len(obstacles) > self.config.max_obstacles:
            return Command(
                diagnostics={
                    "reason": "obstacle_capacity_exceeded",
                    "obstacle_count": len(obstacles),
                }
            )
        try:
            state = self.route.project(ego, self.config.projection_distance)
            if self._recovery_initial is None:
                self._recovery_initial = max(
                    0.0,
                    -float(np.min(self.route.wheel_margins(state, self.geometry))),
                )
            self.planner.recovery = (
                self._recovery_initial,
                self._recovery_ticks * self.config.control_dt,
            )
            self._recovery_ticks += 1
            plan = self.planner.build(state, obstacles, speed_limit, stop_s)
        except ValueError as exc:
            return Command(diagnostics={"reason": str(exc)})
        planning = time.perf_counter() - start
        self.mpc.applied_steer = ego.steer
        cmd = self.mpc.solve(state, plan, obstacles, steer_bound)
        lane_margin = float(np.min(self.route.lane_margins(state, self.geometry)))
        cmd.diagnostics.update(
            {
                **self.route.last_projection,
                "planning_s": planning,
                "lane_wheel_margin_m": float(
                    np.min(self.route.wheel_margins(state, self.geometry))
                ),
                "s_m": float(state[0]),
                "d_m": float(state[1]),
                "epsi_rad": float(state[2]),
                "speed_mps": ego.speed,
                "information_source": "caller_supplied",
                "lane_body_margin_m": lane_margin,
                "initial_body_inside_route_tube": lane_margin >= 0,
            }
        )
        elapsed = time.perf_counter() - start
        cmd.diagnostics["controller_s"] = elapsed
        if self.config.enforce_tick_deadline and elapsed > self.config.control_dt:
            cmd.emergency = True
            cmd.diagnostics["reason"] = "controller_deadline_missed"
        return cmd
