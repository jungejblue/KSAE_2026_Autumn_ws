"""Frame-based episodic command delay; no simulator tick or wall-clock sleep."""

import copy
import math
from collections import deque
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class DelayConfig:
    delay_ms: int = 200
    onset_s: float = 5.0
    duration_s: float = 2.0

    def __post_init__(self):
        if type(self.delay_ms) is not int or self.delay_ms not in (0, 100, 200, 500):
            raise ValueError("delay_ms must be 0, 100, 200 or 500")
        for name, value in (("onset", self.onset_s), ("duration", self.duration_s)):
            if not math.isfinite(value) or value < 0 or abs(value * 20 - round(value * 20)) > 1e-8:
                raise ValueError(f"{name} must be nonnegative and a multiple of 0.05 s")
        if self.duration_s <= 0:
            raise ValueError("duration must be positive")

    @property
    def ticks(self):
        return self.delay_ms // 50

    @property
    def onset_ticks(self):
        return round(self.onset_s * 20)

    @property
    def duration_ticks(self):
        return round(self.duration_s * 20)

    def to_dict(self):
        return asdict(self)


class ActionDelay:
    """FIFO by generation order; release ready prefix and submit only its newest command.

    A zero-delay command cannot overtake an older pending delayed command.
    At the recovery boundary multiple commands may become available at once;
    superseded commands in that batch are dropped. Repeated holds are explicit.
    State export concerns this channel only, not a full simulator checkpoint.
    """

    def __init__(self, config):
        self.config = config
        self.first_frame = self.last_frame = None
        self.queue = deque()
        self.last_selected = None

    def step(self, frame, control, source_frame):
        if type(frame) is not int or type(source_frame) is not int or source_frame > frame:
            raise ValueError("Invalid frame/source frame")
        if self.last_frame is not None and frame != self.last_frame + 1:
            raise ValueError("Delay requires consecutive frames")
        if self.first_frame is None:
            self.first_frame = frame
        index = frame - self.first_frame
        active = (
            self.config.onset_ticks
            <= index
            < (self.config.onset_ticks + self.config.duration_ticks)
        )
        requested = self.config.ticks if active else 0
        self.queue.append(
            {
                "frame": frame,
                "source_frame": source_frame,
                "ready_frame": frame + requested,
                "control": copy.deepcopy(control),
            }
        )
        released = []
        while self.queue and self.queue[0]["ready_frame"] <= frame:
            released.append(self.queue.popleft())
        if released:
            self.last_selected = released[-1]
        selected = self.last_selected
        self.last_frame = frame
        # Only relevant to episodes starting at the first call before any command exists.
        initial = {
            "throttle": 0.0,
            "brake": 1.0,
            "steer": 0.0,
            "hand_brake": False,
            "reverse": False,
            "manual_gear_shift": False,
            "gear": 0,
        }
        result = {
            "schema_version": 1,
            "route_tick": index,
            "episode_active": active,
            "requested_delay_ticks": requested,
            "selected_generation_frame": selected["frame"] if selected else None,
            "selected_source_frame": selected["source_frame"] if selected else None,
            "actual_age_ticks": frame - selected["frame"] if selected else None,
            "queue_depth": len(self.queue),
            "held": not released,
            "initial_hold": selected is None,
            "released_frames": [x["frame"] for x in released],
            "superseded_frames": [x["frame"] for x in released[:-1]],
            "output_control": copy.deepcopy(selected["control"] if selected else initial),
        }
        return result

    def snapshot(self):
        return copy.deepcopy(
            {
                "schema_version": 1,
                "config": self.config.to_dict(),
                "first_frame": self.first_frame,
                "last_frame": self.last_frame,
                "queue": list(self.queue),
                "last_selected": self.last_selected,
            }
        )

    @classmethod
    def restore(cls, state):
        if state.get("schema_version") != 1:
            raise ValueError("Unsupported delay snapshot")
        state = copy.deepcopy(state)
        result = cls(DelayConfig(**state["config"]))
        result.first_frame, result.last_frame = state["first_frame"], state["last_frame"]
        result.queue = deque(state["queue"])
        result.last_selected = state["last_selected"]
        return result
