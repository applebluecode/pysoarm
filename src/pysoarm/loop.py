"""Fixed-rate control-loop timing and error policy.

This module owns one thing: the loop around a read/callback/write cycle. It does not know
about serial ports, calibration, goals or safety guards — those arrive as callables. That
keeps the deadline arithmetic, latency accounting and error policy testable with fake clocks
and plain functions, without constructing an :class:`~pysoarm.arm.SOArm`.

The caller remains responsible for the meaning of a goal and for the bus transaction. This
module only guarantees that:

* deadlines are absolute, so jitter does not accumulate;
* a missed deadline is counted instead of hidden by shortening the next period;
* ``read_state_every`` turns a fresh read into an explicit, documented choice;
* the ``abort`` / ``hold`` / ``skip`` policy is applied uniformly to reads, callbacks and
  writes, with a consecutive-error cutoff.
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Literal

from pysoarm.exceptions import ConfigurationError
from pysoarm.state import ArmState

__all__ = ["ErrorPolicy", "LoopStats", "run_loop", "validate_loop_options"]

#: What to do when a tick raises.
ErrorPolicy = Literal["abort", "hold", "skip"]


@dataclass(frozen=True, slots=True)
class LoopStats:
    """Summary of a finished control loop.

    Args:
        ticks: Number of completed ticks (including ticks that ended in a handled error).
        overruns: Ticks that could not be finished within the period. This is also the
            number of missed deadlines: the loop keeps absolute deadlines, so a missed tick
            is counted rather than hidden by shortening the next period.
        elapsed_s: Wall-clock duration of the loop.
        errors: Ticks that raised and were handled by a non-``abort`` error policy.
        consecutive_errors_max: Longest run of consecutive failed ticks.
        latencies_s: Per-tick command latency, measured from the callback's sampling instant
            to the completed goal write. Only ticks that wrote a goal are recorded.

    All derived values are computed on demand, so the object stays a cheap immutable record.
    """

    ticks: int
    overruns: int
    elapsed_s: float
    errors: int = 0
    consecutive_errors_max: int = 0
    latencies_s: tuple[float, ...] = ()

    @property
    def mean_period_s(self) -> float:
        """Average tick period in seconds (``0.0`` for an empty loop)."""
        return self.elapsed_s / self.ticks if self.ticks else 0.0

    @property
    def mean_hz(self) -> float:
        """Average tick rate in Hz (``0.0`` for an empty loop)."""
        return 1.0 / self.mean_period_s if self.mean_period_s > 0 else 0.0

    @property
    def overrun_ratio(self) -> float:
        """Fraction of ticks that overran their budget."""
        return self.overruns / self.ticks if self.ticks else 0.0

    @property
    def deadline_misses(self) -> int:
        """Alias for :attr:`overruns`, phrased as the real-time metric it is."""
        return self.overruns

    @property
    def mean_latency_s(self) -> float:
        """Mean tick latency in seconds (``0.0`` when no goal was written)."""
        return statistics.fmean(self.latencies_s) if self.latencies_s else 0.0

    def latency_percentile(self, fraction: float) -> float:
        """Latency percentile in seconds.

        Args:
            fraction: ``0.0``..``1.0``; e.g. ``0.95`` for p95.

        Returns:
            The nearest-rank percentile, or ``0.0`` when no goal was written.

        Raises:
            ConfigurationError: If ``fraction`` is outside ``[0, 1]``.
        """
        if not 0.0 <= fraction <= 1.0:
            raise ConfigurationError("fraction must be between 0 and 1")
        if not self.latencies_s:
            return 0.0
        ordered = sorted(self.latencies_s)
        index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
        return ordered[index]

    @property
    def p95_latency_s(self) -> float:
        """95th-percentile tick latency in seconds."""
        return self.latency_percentile(0.95)

    @property
    def max_latency_s(self) -> float:
        """Worst tick latency in seconds."""
        return max(self.latencies_s, default=0.0)

    def as_dict(self) -> dict[str, float | int]:
        """JSON-serialisable summary, handy for benchmarks and logs."""
        return {
            "ticks": self.ticks,
            "overruns": self.overruns,
            "deadline_misses": self.deadline_misses,
            "elapsed_s": self.elapsed_s,
            "mean_hz": self.mean_hz,
            "mean_period_s": self.mean_period_s,
            "errors": self.errors,
            "consecutive_errors_max": self.consecutive_errors_max,
            "mean_latency_s": self.mean_latency_s,
            "p95_latency_s": self.p95_latency_s,
            "max_latency_s": self.max_latency_s,
        }


def validate_loop_options(
    *,
    hz: float,
    read_state_every: int | None,
    on_error: ErrorPolicy,
    max_consecutive_errors: int,
    initial_state: ArmState | None,
) -> None:
    """Validate the loop contract before any timing state is touched.

    Args:
        hz: Target loop rate.
        read_state_every: Read cadence; ``0`` or ``None`` reuse ``initial_state``.
        on_error: Error policy.
        max_consecutive_errors: Handled-error cutoff.
        initial_state: Cached state, required when the loop will not read.

    Raises:
        ConfigurationError: On a non-positive rate, negative cadence, unknown policy,
            non-positive cutoff, or a read-less loop with no cached state.
    """
    if hz <= 0:
        raise ConfigurationError("hz must be positive")
    if read_state_every is not None and read_state_every < 0:
        raise ConfigurationError("read_state_every must be non-negative")
    if on_error not in ("abort", "hold", "skip"):
        raise ConfigurationError(
            f"unknown on_error {on_error!r}; expected 'abort', 'hold' or 'skip'"
        )
    if max_consecutive_errors < 1:
        raise ConfigurationError("max_consecutive_errors must be at least 1")
    if not read_state_every and initial_state is None:
        raise ConfigurationError("read_state_every=0 needs a cached state; call read_once() first")


def run_loop(
    *,
    callback: Callable[[ArmState, float], Sequence[float] | None],
    read_state: Callable[[], ArmState],
    write_goal: Callable[[Sequence[float]], Sequence[float]],
    initial_state: ArmState | None,
    hz: float,
    duration: float | None,
    read_state_every: int | None,
    on_error: ErrorPolicy,
    max_consecutive_errors: int,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> LoopStats:
    """Run a fixed-rate read/callback/write loop.

    Args:
        callback: ``(state, elapsed_s) -> goal_deg | None``.
        read_state: Read and return one state snapshot.
        write_goal: Send one goal and return the goal actually written.
        initial_state: Cached state used when ``read_state_every`` is ``0``/``None`` or as
            the first sample before the first read.
        hz: Target loop rate.
        duration: Stop after this many seconds; ``None`` runs until ``KeyboardInterrupt``.
        read_state_every: Read every ``N`` ticks; ``0``/``None`` never reads.
        on_error: Handle a failed tick with ``"abort"``, ``"hold"`` or ``"skip"``.
        max_consecutive_errors: Abort after this many consecutive handled errors.
        clock: Monotonic clock.
        sleep: Sleep function.

    Returns:
        :class:`LoopStats` with ticks, overruns, errors and latency percentiles.

    Raises:
        ConfigurationError: If an option is invalid (see :func:`validate_loop_options`).
        Exception: The original exception when ``on_error="abort"`` or the cutoff is hit.
    """
    validate_loop_options(
        hz=hz,
        read_state_every=read_state_every,
        on_error=on_error,
        max_consecutive_errors=max_consecutive_errors,
        initial_state=initial_state,
    )
    period = 1.0 / hz
    t0 = clock()
    ticks = 0
    overruns = 0
    errors = 0
    consecutive = 0
    consecutive_max = 0
    latencies: list[float] = []
    state = initial_state
    try:
        while True:
            elapsed = clock() - t0
            if duration is not None and elapsed >= duration:
                break
            try:
                if read_state_every and (ticks % read_state_every == 0 or state is None):
                    state = read_state()
                assert state is not None  # guaranteed by the option validation
                sampled_at = clock()  # the state handed to the callback
                goal = callback(state, elapsed)
                if goal is not None:
                    write_goal(goal)
                    latencies.append(clock() - sampled_at)
                consecutive = 0
            except Exception:
                errors += 1
                consecutive += 1
                consecutive_max = max(consecutive_max, consecutive)
                if on_error == "abort" or consecutive >= max_consecutive_errors:
                    raise
                # "hold" leaves the servos holding their last goal; "skip" just discards
                # the tick. Neither writes a goal derived from a failed sample.
            deadline = t0 + (ticks + 1) * period
            slack = deadline - clock()
            if slack > 0:
                sleep(slack)
            else:
                overruns += 1
            ticks += 1
    except KeyboardInterrupt:
        pass
    return LoopStats(
        ticks=ticks,
        overruns=overruns,
        elapsed_s=clock() - t0,
        errors=errors,
        consecutive_errors_max=consecutive_max,
        latencies_s=tuple(latencies),
    )
