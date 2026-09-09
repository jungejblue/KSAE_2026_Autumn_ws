"""Measured-acceleration feedback around a provisional inverse actuator model.

The gains are initial values for the MKZ fixture, NOT a CARLA calibration result.
Uses forward speed from consecutive snapshots. No simulator state is overwritten.
"""

import math

import numpy as np


class LongitudinalActuator:
    def __init__(self, config):
        self.c = config
        self.reset()

    def reset(self):
        self.previous_speed = None
        self.previous_request = 0.0
        self.filtered_acceleration = 0.0
        self.integral = 0.0
        self.previous_effort = None
        self.holding = False
        self.coast_active = False

    def step(self, acceleration, target_speed, speed, urgent_brake=False, stop_requested=False,
             coast_requested=None):
        c = self.c
        if not all(math.isfinite(v) for v in (acceleration, target_speed, speed)):
            raise ValueError("invalid_actuator_state")
        measured = None
        if self.previous_speed is not None:
            measured = (speed - self.previous_speed) / c.control_dt
            alpha = c.control_dt / (c.acceleration_filter_s + c.control_dt)
            self.filtered_acceleration += alpha * (measured - self.filtered_acceleration)
            # The measured acceleration belongs to the previous applied command.
            error = self.previous_request - self.filtered_acceleration
            self.integral = float(
                np.clip(self.integral + c.acceleration_ki * error * c.control_dt, -1.5, 1.5)
            )
        else:
            error = 0.0
        request = float(np.clip(acceleration, -c.brake_decel, c.accel_max))
        effort = request + c.rolling_drag + c.acceleration_kp * error + self.integral
        unlimited_effort = effort
        # Acceleration-command slew alone does not bound the PI correction.
        # Limit ordinary positive effort after feedback. Any deceleration request
        # or urgent safety braking bypasses this limiter, including throttle release.
        limit_active = (
            c.actuator_effort_rate > 0
            and self.previous_effort is not None
            and self.previous_effort >= 0
            and effort >= 0
            and request >= 0
            and not urgent_brake
        )
        if limit_active:
            delta = c.actuator_effort_rate * c.control_dt
            effort = float(
                np.clip(effort, self.previous_effort - delta, self.previous_effort + delta)
            )
        slew_limited = abs(effort - unlimited_effort) > 1e-12
        if slew_limited and (unlimited_effort - effort) * error > 0:
            self.integral = float(
                np.clip(self.integral - c.acceleration_ki * error * c.control_dt, -1.5, 1.5)
            )
        throttle = float(np.clip(effort / c.throttle_gain, 0.0, 0.6))
        brake = float(np.clip(-effort / c.brake_gain, 0.0, 0.25))
        # Production takes the mode of the selected, constraint-checked rollout.
        # A speed threshold must not independently cancel it after a rebound.
        # Legacy callers without a rollout flag use latched planned-stop mode.
        if coast_requested is None:
            terminal_stop = bool(stop_requested and request < 0 and not urgent_brake and (
                self.coast_active or speed < c.terminal_coast_speed))
        else:
            terminal_stop = bool(coast_requested and not urgent_brake)
        self.coast_active = terminal_stop
        if terminal_stop:
            throttle, brake = 0.0, 0.0
            self.integral = 0.0
        hold = ((target_speed < 0.05 and speed < 0.1 and request <= 0.0)
                or (stop_requested and speed < (0.3 if self.holding else 0.1)))
        self.holding = hold
        if hold:
            throttle, brake = 0.0, 1.0
            self.integral = 0.0
            self.filtered_acceleration = 0.0
        # Bound wind-up if either actuator is saturated.
        if (throttle >= 0.6 and error > 0) or (brake >= 0.25 and error < 0):
            self.integral = float(
                np.clip(self.integral - c.acceleration_ki * error * c.control_dt, -1.5, 1.5)
            )
        self.previous_speed, self.previous_request = speed, request
        self.previous_effort = throttle * c.throttle_gain - brake * c.brake_gain
        return (
            throttle,
            brake,
            dict(
                requested_acceleration_mps2=request,
                measured_acceleration_mps2=measured,
                filtered_acceleration_mps2=self.filtered_acceleration,
                acceleration_error_mps2=error,
                acceleration_integral=self.integral,
                inverse_model_effort_mps2=effort,
                unlimited_effort_mps2=unlimited_effort,
                applied_equivalent_effort_mps2=self.previous_effort,
                effort_slew_limited=slew_limited,
                effort_slew_eligible=limit_active,
                urgent_brake=bool(urgent_brake),
                stationary_hold=hold,
                stop_requested=bool(stop_requested),
                terminal_stop_control=terminal_stop,
                inverse_model_status="provisional_MKZ_gains_require_physical_validation",
            ),
        )
