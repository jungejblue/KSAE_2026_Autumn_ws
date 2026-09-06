"""Action delay and intervention gates; no CARLA imports or vehicle controller."""

from collections import deque
from copy import deepcopy
from typing import Generic, TypeVar

T = TypeVar("T")


class ActionDelayFIFO(Generic[T]):
    """Apply E2E actions after delay_ticks; fill the startup window with initial_action.

    Pass copyable values (such as throttle/steer/brake dictionaries), not opaque
    CARLA extension objects. Fallback commands must bypass this FIFO.
    """

    def __init__(self, delay_ticks: int, initial_action: T) -> None:
        if type(delay_ticks) is not int or delay_ticks < 0:
            raise ValueError("delay_ticks must be a non-negative integer")
        self.delay_ticks = delay_ticks
        self.initial_action = deepcopy(initial_action)
        self._queue: deque[T] = deque()
        self.reset()

    def reset(self) -> None:
        self._queue = deque(deepcopy(self.initial_action) for _ in range(self.delay_ticks))

    def step(self, new_action: T) -> T:
        self._queue.append(deepcopy(new_action))
        return self._queue.popleft()

    def snapshot(self) -> tuple[T, ...]:
        return tuple(deepcopy(list(self._queue)))

    def restore(self, state: tuple[T, ...]) -> None:
        if len(state) != self.delay_ticks:
            raise ValueError("FIFO snapshot length differs from configured delay")
        self._queue = deque(deepcopy(state))


def candidate_time(onset_s: float, latency_ms: int, offset_s: float = 0.5) -> float:
    """Return the candidate timestamp, independently of the monitor decision."""
    if min(onset_s, latency_ms, offset_s) < 0:
        raise ValueError("timing values must be non-negative")
    return onset_s + latency_ms / 1000.0 + offset_s


def baseline_decision(predicted_ttc_risk: bool, predicted_ttlc_risk: bool) -> bool:
    return bool(predicted_ttc_risk or predicted_ttlc_risk)


def proposed_decision(
    *,
    action_age_gate: bool,
    predicted_e2e_risk: bool,
    fallback_benefit_gate: bool,
    persistence_gate: bool,
    emergency_override: bool = False,
    fallback_not_worse: bool = False,
) -> bool:
    """Combine externally computed gates. This function does not estimate TTC/TTLC."""
    normal = action_age_gate and predicted_e2e_risk and fallback_benefit_gate and persistence_gate
    emergency = emergency_override and action_age_gate and predicted_e2e_risk and fallback_not_worse
    return bool(normal or emergency)
