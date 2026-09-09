"""Simulator-independent data contract. Units: m, s, rad; right-handed XY."""

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Config:
    dt: float = 0.15
    control_dt: float = 0.05
    horizon: int = 20
    max_obstacles: int = 6
    cruise_speed: float = 5.0
    max_speed: float = 12.0
    accel_max: float = 2.0
    brake_decel: float = 4.0
    stop_plan_decel: float = 0.7
    stop_control_decel: float = 1.0
    terminal_coast_speed: float = 2.0
    terminal_coast_decel: float = 0.2  # provisional lower coast-deceleration model
    lateral_accel: float = 2.0
    steer_rate: float = 0.7
    cbf_gamma: float = 0.35
    clf_rate: float = 0.8
    clf_weight: float = 1500.0
    lane_margin: float = 0.12
    body_margin_weight: float = 200.0
    obstacle_margin: float = 0.4
    prediction_growth: float = 0.15
    reaction_time: float = 0.3
    stop_buffer: float = 1.0
    recovery_rate: float = 0.35
    recovery_grace: float = 1.0
    recovery_swing_margin: float = 0.15
    recovery_deadline: float = 8.0
    recovery_speed_limit: float = 1.5
    recovery_steer_rate: float = 0.35
    throttle_gain: float = 4.0
    brake_gain: float = 45.0
    rolling_drag: float = 0.2
    acceleration_kp: float = 0.3
    acceleration_ki: float = 0.6
    acceleration_filter_s: float = 0.15
    normal_jerk_limit: float = 4.0
    actuator_effort_rate: float = 4.0  # normal positive-effort slew; 0 disables for ablation
    solver_budget_s: float = 0.045
    max_iter: int = 80
    enforce_tick_deadline: bool = True
    native: bool = False
    backend: str = "sampled"
    feasibility_tol: float = 2e-4
    candidate_offsets: tuple = (0.0, -0.45, 0.45, -0.9, 0.9)
    transition_length: float = 12.0
    projection_distance: float = 6.0
    sensor_range: float = 70.0
    max_snapshot_age: float = 0.1

    def __post_init__(self):
        if self.backend not in ("sampled", "ipopt"):
            raise ValueError("backend must be sampled or ipopt")
        positive = (
            "dt",
            "control_dt",
            "cruise_speed",
            "max_speed",
            "accel_max",
            "brake_decel",
            "stop_plan_decel",
            "stop_control_decel",
            "terminal_coast_speed",
            "terminal_coast_decel",
            "lateral_accel",
            "steer_rate",
            "solver_budget_s",
            "feasibility_tol",
            "transition_length",
            "sensor_range",
            "recovery_deadline",
            "recovery_speed_limit",
            "recovery_steer_rate",
            "throttle_gain",
            "brake_gain",
            "acceleration_filter_s",
            "normal_jerk_limit",
        )
        for name in positive:
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.cbf_gamma <= 1 or not 0 <= self.clf_rate * self.dt < 1:
            raise ValueError("Invalid CBF gamma / CLF decay")
        if self.horizon < 3 or self.max_obstacles < 1 or self.max_iter < 1:
            raise ValueError("Invalid solver dimensions")
        if self.control_dt > self.dt or self.cruise_speed > self.max_speed:
            raise ValueError("control_dt <= dt and cruise_speed <= max_speed required")
        if not self.stop_plan_decel <= self.stop_control_decel <= self.brake_decel:
            raise ValueError("stop_plan_decel <= stop_control_decel <= brake_decel required")
        for name in (
            "lane_margin",
            "body_margin_weight",
            "actuator_effort_rate",
            "obstacle_margin",
            "prediction_growth",
            "reaction_time",
            "stop_buffer",
            "recovery_rate",
            "recovery_grace",
            "recovery_swing_margin",
            "rolling_drag",
            "acceleration_kp",
            "acceleration_ki",
            "max_snapshot_age",
        ):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"Invalid {name}")

    @property
    def terminal_coast_reserve(self):
        return self.terminal_coast_speed ** 2 / (2 * self.terminal_coast_decel)

    @property
    def times(self):
        return np.r_[0.0, self.control_dt + np.arange(self.horizon) * self.dt]


@dataclass(frozen=True)
class Geometry:
    wheelbase: float
    rear_axle_x: float  # rear axle relative to CARLA actor origin, forward positive
    front: float  # bounding box front relative to rear axle
    rear: float  # rear overhang, positive
    half_width: float
    max_steer: float
    box_y: float = 0.0  # RH lateral bounding box offset
    wheels: tuple = ()  # actual wheel centers (forward,left) relative to rear axle

    def __post_init__(self):
        vals = (self.wheelbase, self.front, self.rear, self.half_width, self.max_steer)
        if not all(math.isfinite(v) and v > 0 for v in vals):
            raise ValueError("Invalid vehicle geometry")
        if self.max_steer >= 1.4:
            raise ValueError("Unphysical steering bound")

    @property
    def body_points(self):
        return np.array(
            [
                (x, self.box_y + y)
                for x in (-self.rear, self.front)
                for y in (-self.half_width, self.half_width)
            ]
        )

    @property
    def discs(self):
        length = self.front + self.rear
        spacing = length / 3
        xs = -self.rear + spacing * (np.arange(3) + 0.5)
        return xs, math.hypot(spacing / 2, self.half_width)


@dataclass(frozen=True)
class EgoState:
    x: float
    y: float
    yaw: float
    speed: float
    steer: float = 0.0  # applied road-wheel angle, not normalized command


@dataclass(frozen=True)
class Obstacle:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float

    def at(self, t):
        return np.array([self.x + self.vx * t, self.y + self.vy * t])


@dataclass
class Plan:
    reference: np.ndarray  # N+1 by 4: s, d, epsi, v
    mode: str
    offset: float
    stop_s: float
    diagnostics: dict = field(default_factory=dict)


@dataclass
class Command:
    acceleration: float = 0.0
    steer: float = 0.0
    target_speed: float = 0.0
    emergency: bool = True
    diagnostics: dict = field(default_factory=dict)
    prediction: np.ndarray | None = None
    controls: np.ndarray | None = None
