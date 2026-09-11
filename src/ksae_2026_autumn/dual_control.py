"""Command holding, forced handover/recovery, and observed prefix-state matching."""

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Settings:
    trigger_gap_m: float = 50.0
    trigger_min_speed_mps: float = 6.0
    takeover_after_ticks: int = 15
    stall_ticks: int = 60
    horizon_ticks: int = 100
    hold_ticks: int = 100
    lead_release_ticks: int = 360
    recovery_gap_m: float = 25.0
    fresh_ticks: int = 3
    post_recovery_ticks: int = 60


class Protocol:
    def __init__(self, mode, settings=None):
        if mode not in ("clean", "delayed", "takeover"):
            raise ValueError(mode)
        self.mode, self.c = mode, settings or Settings()
        self.trigger = self.takeover = self.recovery = None
        self.fresh = 0
        self.last_frame = None
        self.stop_seen = False

    def step(self, frame, eligible, gap, speed, fresh, fallback_ok, trigger_override=None):
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError("Nonconsecutive frames")
        self.last_frame = frame
        event = None
        geometric = (
            eligible and 0 < gap <= self.c.trigger_gap_m and speed >= self.c.trigger_min_speed_mps
        )
        trigger_now = geometric if trigger_override is None else trigger_override
        if self.trigger is None and trigger_now:
            self.trigger, event = frame, "fault_onset"
        age = None if self.trigger is None else frame - self.trigger
        released = age is not None and age >= self.c.lead_release_ticks
        if age is not None and not released and speed <= 0.3 and gap > 0:
            self.stop_seen = True
        if self.mode == "takeover" and age is not None:
            if self.takeover is None and age >= self.c.takeover_after_ticks:
                self.takeover, event = frame, "takeover_forced"
            if self.takeover is not None and self.recovery is None:
                ready = (
                    frame - self.takeover >= self.c.hold_ticks
                    and released
                    and gap >= self.c.recovery_gap_m
                    and fresh
                    and fallback_ok
                )
                self.fresh = self.fresh + 1 if ready else 0
                if self.fresh >= self.c.fresh_ticks:
                    self.recovery, event = frame, "recovery"
        return {
            "selected": "FALLBACK"
            if self.takeover is not None and self.recovery is None
            else "E2E",
            "event": event,
            "trigger_frame": self.trigger,
            "takeover_frame": self.takeover,
            "recovery_frame": self.recovery,
            "fresh_count": self.fresh,
            "lead_released": released,
            "stop_seen_before_release": self.stop_seen,
        }


class Stall:
    def __init__(self, enabled, duration_ticks=60):
        self.enabled = enabled
        self.duration = duration_ticks
        self.onset = None
        self.previous = None
        self.held = None
        self.last_frame = None

    def step(self, frame, control, source_frame, onset=False):
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError("Nonconsecutive stall frames")
        self.last_frame = frame
        if onset:
            if self.onset is not None or self.previous is None:
                raise ValueError("Stall needs exactly one onset and a prior command")
            self.onset = frame
            self.held = copy.deepcopy(self.previous)
        current = (copy.deepcopy(control), frame, source_frame)
        active = self.enabled and self.onset is not None and frame < self.onset + self.duration
        selected = self.held if active else current
        self.previous = current
        return {
            "fault_kind": "hold_last_command_compute_stall",
            "episode_active": active,
            "output_control": copy.deepcopy(selected[0]),
            "selected_generation_frame": selected[1],
            "selected_source_frame": selected[2],
            "actual_age_ticks": frame - selected[1],
            "requested_delay_ticks": None,
            "duration_ticks": self.duration,
        }


LIMITS = {
    "position_m": 0.05,
    "velocity_mps": 0.1,
    "yaw_rad": math.radians(0.5),
    "angular_velocity_radps": 0.05,
    "acceleration_mps2": 0.5,
}


def state_error(current, reference):
    def actor(a, b):
        if a["type_id"] != b["type_id"]:
            raise ValueError("Actor type changed")
        return {
            "position_m": math.dist(a["position_m"], b["position_m"]),
            "velocity_mps": math.dist(a["velocity_mps"], b["velocity_mps"]),
            "yaw_rad": abs(
                math.atan2(
                    math.sin(a["rpy_rad"][2] - b["rpy_rad"][2]),
                    math.cos(a["rpy_rad"][2] - b["rpy_rad"][2]),
                )
            ),
            "angular_velocity_radps": math.dist(
                a["angular_velocity_radps"], b["angular_velocity_radps"]
            ),
            "acceleration_mps2": math.dist(a["acceleration_mps2"], b["acceleration_mps2"]),
        }

    if len(current["actors"]) != 1 or len(reference["actors"]) != 1:
        raise ValueError("Fixture must contain exactly one non-ego lead actor")
    errors = [
        actor(current["ego"], reference["ego"]),
        actor(current["actors"][0], reference["actors"][0]),
    ]
    maxima = {key: max(e[key] for e in errors) for key in LIMITS}
    return {
        "maxima": maxima,
        "within_tolerance": all(maxima[k] <= LIMITS[k] for k in LIMITS),
    }


class Prefix:
    def __init__(self, filename):
        self.path = Path(filename)
        self.rows = [json.loads(s) for s in self.path.read_text().splitlines() if s.strip()]
        if not self.rows or any(
            b["frame"] != a["frame"] + 1 for a, b in zip(self.rows, self.rows[1:], strict=False)
        ):
            raise ValueError("Invalid reference prefix")
        self.first = self.rows[0]["frame"]
        triggers = [
            r["frame"] - self.first
            for r in self.rows
            if r.get("dual", r.get("restart", {}))["event"] == "fault_onset"
        ]
        if len(triggers) != 1:
            raise ValueError("Reference must contain exactly one fault onset")
        self.onset_tick = triggers[0]

    def row(self, tick):
        if tick >= len(self.rows):
            raise ValueError("Missing reference prefix row")
        return self.rows[tick]
