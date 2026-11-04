"""Joint-space trajectories.

All trajectories are pure functions of time: they answer "where should the joints be at time
``t``" and know nothing about the bus. Timing (when to call ``sample``) belongs to the control
loop, so a trajectory can be tested, replayed or rendered without hardware.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise
from typing import Protocol, runtime_checkable

from pysoarm.exceptions import ConfigurationError

__all__ = ["LinearRamp", "MinimumJerk", "ResampledTrajectory", "Trajectory"]

#: A resampled table needs at least a start and an end sample.
_MIN_SAMPLES = 2


@runtime_checkable
class Trajectory(Protocol):
    """Anything that can be sampled in joint space."""

    @property
    def duration(self) -> float:
        """Total duration in seconds."""
        ...

    def sample(self, t: float) -> tuple[float, ...]:
        """Joint goal at time ``t`` (clamped to ``[0, duration]``)."""
        ...


def _validate_endpoints(
    start: Sequence[float], end: Sequence[float], duration: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if len(start) != len(end):
        raise ConfigurationError(f"start has {len(start)} joints, end has {len(end)}")
    if not start:
        raise ConfigurationError("trajectory needs at least one joint")
    if duration <= 0:
        raise ConfigurationError("duration must be positive")
    return tuple(float(v) for v in start), tuple(float(v) for v in end)


def _progress(t: float, duration: float) -> float:
    if t <= 0:
        return 0.0
    if t >= duration:
        return 1.0
    return t / duration


def _lerp(start: tuple[float, ...], end: tuple[float, ...], s: float) -> tuple[float, ...]:
    return tuple(a + (b - a) * s for a, b in zip(start, end, strict=True))


@dataclass(frozen=True, slots=True)
class MinimumJerk:
    """Quintic minimum-jerk profile ``10s^3 - 15s^4 + 6s^5``.

    Smooth position, velocity and acceleration, zero velocity at both ends. The default for
    :meth:`SOArm.move_to` because it is gentle on geared servos.

    Args:
        start: Joint values at ``t = 0``.
        end: Joint values at ``t = duration``.
        duration: Duration in seconds.
    """

    start: tuple[float, ...]
    end: tuple[float, ...]
    duration: float

    def __post_init__(self) -> None:
        start, end = _validate_endpoints(self.start, self.end, self.duration)
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)

    def sample(self, t: float) -> tuple[float, ...]:
        """Joint goal at time ``t`` (clamped to the trajectory's time span)."""
        s = _progress(t, self.duration)
        s = s * s * s * (10.0 + s * (-15.0 + 6.0 * s))
        return _lerp(self.start, self.end, s)

    def __call__(self, t: float) -> tuple[float, ...]:
        """Alias of :meth:`sample`, so a trajectory is callable like a function."""
        return self.sample(t)


@dataclass(frozen=True, slots=True)
class LinearRamp:
    """Constant-velocity interpolation, for debugging and short moves.

    Args:
        start: Joint values at ``t = 0``.
        end: Joint values at ``t = duration``.
        duration: Duration in seconds.
    """

    start: tuple[float, ...]
    end: tuple[float, ...]
    duration: float

    def __post_init__(self) -> None:
        start, end = _validate_endpoints(self.start, self.end, self.duration)
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)

    def sample(self, t: float) -> tuple[float, ...]:
        """Joint goal at time ``t`` (clamped to the trajectory's time span)."""
        return _lerp(self.start, self.end, _progress(t, self.duration))

    def __call__(self, t: float) -> tuple[float, ...]:
        """Alias of :meth:`sample`, so a trajectory is callable like a function."""
        return self.sample(t)


@dataclass(frozen=True, slots=True)
class ResampledTrajectory:
    """Piecewise-linear replay of a recorded ``(t, q)`` table.

    This is the replay *primitive*: it consumes any time-stamped joint table (a ``.npz``, a
    CSV, a LeRobot parquet — your loader, your choice) and hands back joint goals. No dataset
    format is assumed or produced.

    Args:
        times: Strictly increasing timestamps in seconds, starting at or after 0.
        positions: One joint tuple per timestamp, all the same length.

    Raises:
        ConfigurationError: If the table is empty, misaligned or not strictly increasing.
    """

    times: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        times = tuple(float(t) for t in self.times)
        positions = tuple(tuple(float(v) for v in row) for row in self.positions)
        if len(times) < _MIN_SAMPLES:
            raise ConfigurationError("a recorded trajectory needs at least two samples")
        if len(times) != len(positions):
            raise ConfigurationError(f"{len(times)} timestamps but {len(positions)} position rows")
        if any(b <= a for a, b in pairwise(times)):
            raise ConfigurationError("timestamps must be strictly increasing")
        width = len(positions[0])
        if width == 0 or any(len(row) != width for row in positions):
            raise ConfigurationError("all position rows must have the same, non-zero length")
        object.__setattr__(self, "times", times)
        object.__setattr__(self, "positions", positions)

    @property
    def duration(self) -> float:
        """Timestamp of the last sample."""
        return self.times[-1]

    def sample(self, t: float) -> tuple[float, ...]:
        """Linearly interpolated joint goal at time ``t``."""
        if t <= self.times[0]:
            return self.positions[0]
        if t >= self.times[-1]:
            return self.positions[-1]
        for index in range(1, len(self.times)):
            if t <= self.times[index]:
                t0, t1 = self.times[index - 1], self.times[index]
                span = t1 - t0
                s = 0.0 if span <= 0 else (t - t0) / span
                return _lerp(self.positions[index - 1], self.positions[index], math.fsum([s]))
        return self.positions[-1]

    def __call__(self, t: float) -> tuple[float, ...]:
        """Alias of :meth:`sample`, so a trajectory is callable like a function."""
        return self.sample(t)
