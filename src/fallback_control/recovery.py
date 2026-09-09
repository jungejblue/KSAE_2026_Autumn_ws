"""Finite, activation-anchored recovery budget; never changes the V detector."""

import numpy as np


def recovery_envelope(times, initial_allowance, config):
    t = np.maximum(0.0, np.asarray(times, float))
    if initial_allowance <= 0:
        return np.zeros_like(t)
    # A rear overhang initially swings outward during inward steering. Permit a
    # bounded transient only in recovery, then require zero allowance by deadline.
    baseline = initial_allowance * np.exp(
        -config.recovery_rate * np.maximum(0.0, t - config.recovery_grace)
    )
    swing = (
        config.recovery_swing_margin
        * np.minimum(t / 0.5, 1.0)
        * np.exp(-np.maximum(0.0, t - 1.0) / 0.8)
    )
    taper = np.clip((config.recovery_deadline - t) / 2.0, 0.0, 1.0)
    return (baseline + swing) * taper
