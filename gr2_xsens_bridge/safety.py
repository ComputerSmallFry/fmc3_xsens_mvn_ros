"""Safety utilities: clamping, rate limiting, filtering, timeout detection."""

from __future__ import annotations

import time
from typing import Dict, List

from . import config


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def clamp_to_limits(cmd: Dict[str, List[float]]) -> Dict[str, List[float]]:
    """Clamp every joint value to its configured limits."""
    out: Dict[str, List[float]] = {}
    for group, values in cmd.items():
        limits = config.JOINT_LIMITS.get(group)
        if limits is None:
            out[group] = list(values)
            continue
        out[group] = [
            clamp(v, lo, hi) for v, (lo, hi) in zip(values, limits)
        ]
    return out


def rate_limit(
    target: Dict[str, List[float]],
    previous: Dict[str, List[float]],
    dt: float,
) -> Dict[str, List[float]]:
    """Limit per-joint angular velocity between consecutive commands."""
    out: Dict[str, List[float]] = {}
    for group, tgt in target.items():
        prev = previous.get(group, [0.0] * len(tgt))
        max_vels = config.MAX_JOINT_VEL.get(group, [999.0] * len(tgt))
        clamped = []
        for t, p, mv in zip(tgt, prev, max_vels):
            max_step = mv * dt
            delta = clamp(t - p, -max_step, max_step)
            clamped.append(p + delta)
        out[group] = clamped
    return out


def low_pass_filter(
    new_cmd: Dict[str, List[float]],
    prev_filtered: Dict[str, List[float]],
    alpha: float = config.SMOOTHING_ALPHA,
) -> Dict[str, List[float]]:
    """Simple exponential moving average per joint."""
    out: Dict[str, List[float]] = {}
    for group, new_vals in new_cmd.items():
        prev = prev_filtered.get(group, [0.0] * len(new_vals))
        out[group] = [
            alpha * n + (1.0 - alpha) * p for n, p in zip(new_vals, prev)
        ]
    return out


def lerp_cmd(
    start: Dict[str, List[float]],
    end: Dict[str, List[float]],
    t: float,
) -> Dict[str, List[float]]:
    """Linear interpolation between two command dicts, t in [0, 1]."""
    out: Dict[str, List[float]] = {}
    for group in end:
        s = start.get(group, [0.0] * len(end[group]))
        e = end[group]
        out[group] = [sv + t * (ev - sv) for sv, ev in zip(s, e)]
    return out


class DataTimeoutMonitor:
    """Detect when Xsens data stops arriving."""

    def __init__(self, timeout_sec: float = config.DATA_TIMEOUT_SEC) -> None:
        self.timeout = timeout_sec
        self.last_data_time = time.monotonic()
        self.timed_out = False

    def feed(self) -> None:
        self.last_data_time = time.monotonic()
        self.timed_out = False

    def check(self) -> bool:
        if time.monotonic() - self.last_data_time > self.timeout:
            self.timed_out = True
        return self.timed_out
