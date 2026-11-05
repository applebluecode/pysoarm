"""Pure control-loop tests: deadlines, cadence, latency and error policy.

These tests exercise :func:`pysoarm.loop.run_loop` with injected functions and a fake clock,
so they need neither an ``SOArm`` nor a bus. The arm-level integration tests remain in
``test_arm.py`` to prove the binding still works.
"""

from __future__ import annotations

from collections.abc import Sequence

import pytest

from pysoarm import ArmState, LoopStats
from pysoarm.exceptions import ConfigurationError
from pysoarm.loop import run_loop, validate_loop_options


class FakeClock:
    """Deterministic monotonic clock whose ``sleep`` advances time."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(0.0, seconds)


def state(position: float = 0.0) -> ArmState:
    """Minimal snapshot; the loop only needs a value to hand to the callback."""
    return ArmState(
        q_deg=(position,) * 6,
        speed_raw=(0,) * 6,
        current=(0,) * 6,
        temperature_c=(35,) * 6,
        voltage_v=(12.0,) * 6,
        t=0.0,
    )


def no_goal(_state: ArmState, _elapsed: float) -> None:
    """Callback that never writes a goal."""


def test_loop_paces_and_counts() -> None:
    clock = FakeClock()
    reads = 0

    def read() -> ArmState:
        nonlocal reads
        reads += 1
        return state()

    stats = run_loop(
        callback=no_goal,
        read_state=read,
        write_goal=lambda goal: goal,
        initial_state=None,
        hz=10.0,
        duration=0.35,
        read_state_every=1,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.ticks == 4
    assert stats.overruns == 0
    assert stats.deadline_misses == 0
    assert stats.errors == 0
    assert reads == 4
    assert stats.mean_period_s == pytest.approx(0.1)
    assert stats.mean_hz == pytest.approx(10.0)


def test_loop_keeps_absolute_deadlines_and_counts_overruns() -> None:
    clock = FakeClock()

    def slow_read() -> ArmState:
        clock.now += 0.15  # longer than the 0.1 s period
        return state()

    stats = run_loop(
        callback=no_goal,
        read_state=slow_read,
        write_goal=lambda goal: goal,
        initial_state=None,
        hz=10.0,
        duration=0.4,
        read_state_every=1,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.overruns == stats.ticks
    assert stats.overruns > 0
    assert clock.now >= 0.4


def test_read_state_every_n_reads_less_often() -> None:
    clock = FakeClock()
    reads = 0

    def read() -> ArmState:
        nonlocal reads
        reads += 1
        return state()

    stats = run_loop(
        callback=no_goal,
        read_state=read,
        write_goal=lambda goal: goal,
        initial_state=None,
        hz=10.0,
        duration=0.55,
        read_state_every=2,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.ticks == 6
    assert reads == 3


def test_cached_state_never_reads() -> None:
    clock = FakeClock()

    def forbidden() -> ArmState:
        raise AssertionError("read_state must not be called")

    stats = run_loop(
        callback=no_goal,
        read_state=forbidden,
        write_goal=lambda goal: goal,
        initial_state=state(),
        hz=10.0,
        duration=0.25,
        read_state_every=0,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.ticks == 3

    clock = FakeClock()
    stats = run_loop(
        callback=no_goal,
        read_state=forbidden,
        write_goal=lambda goal: goal,
        initial_state=state(),
        hz=10.0,
        duration=0.25,
        read_state_every=None,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.ticks == 3


def test_latency_is_measured_from_callback_sampling_to_write() -> None:
    clock = FakeClock()

    def write(goal: Sequence[float]) -> Sequence[float]:
        clock.sleep(0.02)
        return goal

    stats = run_loop(
        callback=lambda s, t: s.q_deg,
        read_state=state,
        write_goal=write,
        initial_state=None,
        hz=10.0,
        duration=0.25,
        read_state_every=1,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.mean_latency_s == pytest.approx(0.02)
    assert stats.p95_latency_s == pytest.approx(0.02)
    assert stats.max_latency_s == pytest.approx(0.02)
    assert stats.as_dict()["mean_latency_s"] == pytest.approx(0.02)


def test_error_policy_hold_and_skip_continue() -> None:
    for policy in ("hold", "skip"):
        clock = FakeClock()
        calls = 0

        def flaky(s: ArmState, t: float) -> None:
            nonlocal calls
            calls += 1
            if calls == 1:
                raise RuntimeError("glitch")

        stats = run_loop(
            callback=flaky,
            read_state=state,
            write_goal=lambda goal: goal,
            initial_state=None,
            hz=10.0,
            duration=0.35,
            read_state_every=1,
            on_error=policy,
            max_consecutive_errors=10,
            clock=clock,
            sleep=clock.sleep,
        )
        assert stats.errors == 1
        assert stats.consecutive_errors_max == 1


def test_error_policy_abort_and_cutoff() -> None:
    def always_fails(s: ArmState, t: float) -> None:
        raise RuntimeError("glitch")

    common = {
        "callback": always_fails,
        "read_state": state,
        "write_goal": lambda goal: goal,
        "initial_state": None,
        "hz": 10.0,
        "duration": 1.0,
        "read_state_every": 1,
        "max_consecutive_errors": 2,
        "clock": FakeClock(),
        "sleep": lambda _seconds: None,
    }
    with pytest.raises(RuntimeError, match="glitch"):
        run_loop(on_error="abort", **common)  # type: ignore[arg-type]
    with pytest.raises(RuntimeError, match="glitch"):
        run_loop(on_error="skip", **common)  # type: ignore[arg-type]


def test_keyboard_interrupt_returns_stats() -> None:
    clock = FakeClock()
    reads = 0

    def read() -> ArmState:
        nonlocal reads
        reads += 1
        if reads == 2:
            raise KeyboardInterrupt
        return state()

    stats = run_loop(
        callback=no_goal,
        read_state=read,
        write_goal=lambda goal: goal,
        initial_state=None,
        hz=10.0,
        duration=1.0,
        read_state_every=1,
        on_error="abort",
        max_consecutive_errors=10,
        clock=clock,
        sleep=clock.sleep,
    )
    assert stats.ticks == 1
    assert stats.errors == 0


def test_validation_rejects_bad_options() -> None:
    with pytest.raises(ConfigurationError, match="hz must be positive"):
        validate_loop_options(
            hz=0.0,
            read_state_every=1,
            on_error="abort",
            max_consecutive_errors=3,
            initial_state=None,
        )
    with pytest.raises(ConfigurationError, match="read_state_every"):
        validate_loop_options(
            hz=1.0,
            read_state_every=-1,
            on_error="abort",
            max_consecutive_errors=3,
            initial_state=None,
        )
    with pytest.raises(ConfigurationError, match="unknown on_error"):
        validate_loop_options(
            hz=1.0,
            read_state_every=1,
            on_error="explode",  # type: ignore[arg-type]
            max_consecutive_errors=3,
            initial_state=None,
        )
    with pytest.raises(ConfigurationError, match="max_consecutive_errors"):
        validate_loop_options(
            hz=1.0,
            read_state_every=1,
            on_error="abort",
            max_consecutive_errors=0,
            initial_state=None,
        )
    with pytest.raises(ConfigurationError, match="cached state"):
        validate_loop_options(
            hz=1.0,
            read_state_every=0,
            on_error="abort",
            max_consecutive_errors=3,
            initial_state=None,
        )


def test_errors_are_attributed_to_read_callback_and_write() -> None:
    def bad_read() -> ArmState:
        raise RuntimeError("read")

    def bad_callback(_state: ArmState, _elapsed: float) -> Sequence[float] | None:
        raise RuntimeError("callback")

    def bad_write(_goal: Sequence[float]) -> Sequence[float]:
        raise RuntimeError("write")

    def run(**overrides: object) -> LoopStats:
        clock = FakeClock()
        options: dict[str, object] = {
            "callback": no_goal,
            "read_state": state,
            "write_goal": bad_write,
            "initial_state": state(),
            "hz": 1000.0,
            "duration": 0.005,
            "read_state_every": 1,
            "on_error": "hold",
            "max_consecutive_errors": 100,
            "clock": clock,
            "sleep": clock.sleep,
        }
        options.update(overrides)
        return run_loop(**options)  # type: ignore[arg-type]

    read_stats = run(read_state=bad_read)
    assert read_stats.read_errors == read_stats.ticks
    assert read_stats.callback_errors == read_stats.write_errors == 0

    callback_stats = run(callback=bad_callback)
    assert callback_stats.callback_errors == callback_stats.ticks
    assert callback_stats.read_errors == callback_stats.write_errors == 0

    write_stats = run(callback=lambda s, t: s.q_deg)
    assert write_stats.write_errors == write_stats.ticks
    assert write_stats.read_errors == write_stats.callback_errors == 0
    assert write_stats.errors == (
        write_stats.read_errors + write_stats.callback_errors + write_stats.write_errors
    )
    assert {"read_errors", "callback_errors", "write_errors"} <= set(write_stats.as_dict())


def test_loop_stats_are_bounded() -> None:
    empty = LoopStats(ticks=0, overruns=0, elapsed_s=0.0)
    assert empty.mean_period_s == 0.0
    assert empty.mean_hz == 0.0
    assert empty.overrun_ratio == 0.0
    assert empty.mean_latency_s == 0.0
    assert empty.p95_latency_s == 0.0
    assert empty.max_latency_s == 0.0
    with pytest.raises(ConfigurationError, match="fraction"):
        empty.latency_percentile(1.5)
    stats = LoopStats(ticks=2, overruns=1, elapsed_s=1.0, latencies_s=(0.001, 0.003))
    assert stats.overrun_ratio == 0.5
    assert stats.latency_percentile(0.0) == pytest.approx(0.001)
    assert stats.latency_percentile(0.5) == pytest.approx(0.001)
    assert stats.latency_percentile(1.0) == pytest.approx(0.003)
