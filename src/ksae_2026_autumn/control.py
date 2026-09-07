"""Copyable command queues and boolean switching rules."""

from collections import deque
from copy import deepcopy
from typing import Generic, TypeVar

T = TypeVar("T")


class ActionDelayFIFO(Generic[T]):
    """Return commands after delay_ticks calls; start with copies of initial_action.

    Pass copyable values (such as throttle/steer/brake dictionaries), not opaque
    CARLA extension objects. Enqueue only commands that should be delayed.
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
    """Return onset_s + latency_ms / 1000 + offset_s, in seconds."""
    if min(onset_s, latency_ms, offset_s) < 0:
        raise ValueError("timing values must be non-negative")
    return onset_s + latency_ms / 1000.0 + offset_s


def baseline_decision(predicted_ttc_risk: bool, predicted_ttlc_risk: bool) -> bool:
    """Return whether either collision risk or corridor-departure risk is present."""
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
    """Combine externally supplied booleans into a switching decision.

    Normal switching requires all four gates. With emergency_override enabled,
    action_age_gate, predicted_e2e_risk and fallback_not_worse are sufficient.
    Risk prediction and temporal buffering are performed by the caller.
    """
    normal = action_age_gate and predicted_e2e_risk and fallback_benefit_gate and persistence_gate
    emergency = emergency_override and action_age_gate and predicted_e2e_risk and fallback_not_worse
    return bool(normal or emergency)
