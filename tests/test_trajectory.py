"""Joint-space trajectories: endpoints, monotonicity and resampling."""

from __future__ import annotations

from itertools import pairwise

import pytest

from pysoarm.exceptions import ConfigurationError
from pysoarm.trajectory import LinearRamp, MinimumJerk, ResampledTrajectory

START = (0.0, -10.0, 20.0)
END = (30.0, 10.0, -20.0)


def test_minimum_jerk_hits_both_endpoints() -> None:
    trajectory = MinimumJerk(start=START, end=END, duration=2.0)
    assert trajectory.sample(0.0) == pytest.approx(START)
    assert trajectory.sample(2.0) == pytest.approx(END)
    # outside the span the endpoints are held, not extrapolated
    assert trajectory.sample(-1.0) == pytest.approx(START)
    assert trajectory.sample(99.0) == pytest.approx(END)


def test_minimum_jerk_midpoint_is_halfway() -> None:
    trajectory = MinimumJerk(start=(0.0,), end=(10.0,), duration=1.0)
    # s(0.5) = 10*0.125 - 15*0.0625 + 6*0.03125 = 0.5
    assert trajectory.sample(0.5) == pytest.approx((5.0,))


def test_minimum_jerk_is_monotonic_and_smooth_at_the_ends() -> None:
    trajectory = MinimumJerk(start=(0.0,), end=(10.0,), duration=1.0)
    samples = [trajectory.sample(i / 100.0)[0] for i in range(101)]
    steps = [b - a for a, b in pairwise(samples)]
    assert all(step >= 0 for step in steps)
    # zero velocity at both ends: first/last steps are the smallest
    assert steps[0] < steps[50] and steps[-1] < steps[50]


def test_linear_ramp_is_constant_velocity() -> None:
    trajectory = LinearRamp(start=(0.0,), end=(10.0,), duration=2.0)
    assert trajectory.sample(0.5) == pytest.approx((2.5,))
    assert trajectory.sample(1.0) == pytest.approx((5.0,))


@pytest.mark.parametrize(
    ("start", "end", "duration", "match"),
    [
        ((0.0,), (1.0, 2.0), 1.0, "start has 1 joints, end has 2"),
        ((), (), 1.0, "at least one joint"),
        ((0.0,), (1.0,), 0.0, "duration must be positive"),
    ],
)
def test_invalid_trajectories_are_rejected(
    start: tuple[float, ...], end: tuple[float, ...], duration: float, match: str
) -> None:
    with pytest.raises(ConfigurationError, match=match):
        MinimumJerk(start=start, end=end, duration=duration)


def test_resampled_replay_interpolates_and_clamps() -> None:
    trajectory = ResampledTrajectory(
        times=(0.0, 1.0, 2.0),
        positions=((0.0, 0.0), (10.0, -10.0), (10.0, 0.0)),
    )
    assert trajectory.duration == 2.0
    assert trajectory.sample(-1.0) == (0.0, 0.0)
    assert trajectory.sample(0.5) == pytest.approx((5.0, -5.0))
    assert trajectory.sample(1.5) == pytest.approx((10.0, -5.0))
    assert trajectory.sample(9.0) == (10.0, 0.0)


@pytest.mark.parametrize(
    ("times", "positions", "match"),
    [
        ((0.0,), ((0.0,),), "at least two samples"),
        ((0.0, 1.0), ((0.0,),), "timestamps but 1 position rows"),
        ((0.0, 0.0), ((0.0,), (1.0,)), "strictly increasing"),
        ((0.0, 1.0), ((0.0,), (1.0, 2.0)), "same, non-zero length"),
        ((0.0, 1.0), ((), ()), "same, non-zero length"),
    ],
)
def test_invalid_recorded_tables_are_rejected(
    times: tuple[float, ...],
    positions: tuple[tuple[float, ...], ...],
    match: str,
) -> None:
    with pytest.raises(ConfigurationError, match=match):
        ResampledTrajectory(times=times, positions=positions)


def test_trajectories_are_callable() -> None:
    assert MinimumJerk(start=(0.0,), end=(1.0,), duration=1.0)(1.0) == pytest.approx((1.0,))
    assert LinearRamp(start=(0.0,), end=(1.0,), duration=1.0)(0.0) == pytest.approx((0.0,))
    assert ResampledTrajectory(times=(0.0, 1.0), positions=((0.0,), (1.0,)))(1.0) == (1.0,)
