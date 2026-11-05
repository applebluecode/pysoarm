"""Safety guards: clipping, soft limits, thermal/supply guards and the watchdog."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from pysoarm.calibration import Calibration
from pysoarm.exceptions import ConfigurationError, SafetyViolation, WatchdogTimeout
from pysoarm.models import SO101
from pysoarm.safety import SafetyLimits, apply_guards
from pysoarm.state import ArmState

ZERO = (0.0,) * SO101.joint_count


def make_state(
    *,
    q_deg: tuple[float, ...] = ZERO,
    temperature_c: int = 35,
    voltage_v: float = 11.9,
    failures: int = 0,
    t: float = 1.0,
) -> ArmState:
    n = SO101.joint_count
    return ArmState(
        q_deg=q_deg,
        speed_raw=(0,) * n,
        current=(0,) * n,
        temperature_c=(temperature_c,) * n,
        voltage_v=(voltage_v,) * n,
        t=t,
        consecutive_failures=failures,
    )


@pytest.fixture
def calibration(calibration_path: Path) -> Calibration:
    return Calibration.from_json(calibration_path)


def test_goal_inside_limits_passes_through(
    calibration: Calibration, calibration_dict: dict[str, dict[str, int]]
) -> None:
    del calibration_dict
    report = apply_guards(
        ZERO, state=make_state(), calibration=calibration, model=SO101, limits=SafetyLimits()
    )
    assert report.goal_deg == ZERO
    assert not report.clipped
    assert report.reasons == ()
    assert report.derate == 1.0


def test_soft_limit_clamps_with_margin(
    calibration: Calibration, calibration_dict: dict[str, dict[str, int]]
) -> None:
    # isolate the soft limit: no speed ceiling and a step cap that cannot bind
    limits = SafetyLimits(max_step_deg=1000.0, max_speed_deg_s=None, limit_margin_deg=2.0)
    report = apply_guards(
        (500.0,) * SO101.joint_count,
        state=make_state(),
        calibration=calibration,
        model=SO101,
        limits=limits,
    )
    gripper = calibration.for_joint("gripper")
    expected = gripper.raw_to_deg(gripper.range_max) - 2.0
    assert report.goal_deg[-1] == pytest.approx(expected)
    assert report.clipped
    assert any("clamped gripper" in reason for reason in report.reasons)


def test_per_step_clipping_protects_the_first_goal(calibration: Calibration) -> None:
    limits = SafetyLimits(max_step_deg=5.0, limit_margin_deg=0.0)
    state = make_state()
    report = apply_guards(
        (30.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        state=state,
        calibration=calibration,
        model=SO101,
        limits=limits,
        step_deg=5.0,
    )
    assert report.goal_deg[0] == pytest.approx(5.0)
    assert report.clipped
    assert any("clipped shoulder_pan" in reason for reason in report.reasons)


def test_temperature_derates_the_step(calibration: Calibration) -> None:
    limits = SafetyLimits(max_step_deg=5.0, temp_warn_c=55.0, temp_max_c=65.0)
    state = make_state(temperature_c=60, q_deg=(0.0,) * SO101.joint_count)
    report = apply_guards(
        (30.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        state=state,
        calibration=calibration,
        model=SO101,
        limits=limits,
        step_deg=5.0,
    )
    # 60 C is halfway between warn (55) and max (65) -> derate 1 - 0.75/2 = 0.625
    assert report.derate == pytest.approx(0.625)
    assert report.goal_deg[0] == pytest.approx(5.0 * 0.625)


@pytest.mark.parametrize(
    ("state", "match"),
    [
        (make_state(temperature_c=70), "at or above"),
        (make_state(voltage_v=9.5), "below"),
        (make_state(failures=3), "consecutive read failures"),
    ],
)
def test_unhealthy_state_is_refused(calibration: Calibration, state: ArmState, match: str) -> None:
    limits = SafetyLimits(temp_max_c=70.0, voltage_min_v=10.0, max_consecutive_failures=3)
    expected = WatchdogTimeout if "consecutive" in match else SafetyViolation
    with pytest.raises(expected, match=match):
        apply_guards(ZERO, state=state, calibration=calibration, model=SO101, limits=limits)


def test_watchdog_is_a_safety_violation() -> None:
    assert issubclass(WatchdogTimeout, SafetyViolation)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_goals_are_rejected(calibration: Calibration, bad: float) -> None:
    with pytest.raises(SafetyViolation, match="non-finite"):
        apply_guards(
            (bad, 0.0, 0.0, 0.0, 0.0, 0.0),
            state=make_state(),
            calibration=calibration,
            model=SO101,
            limits=SafetyLimits(),
        )


def test_wrong_joint_count_is_rejected(calibration: Calibration) -> None:
    with pytest.raises(ConfigurationError, match="Expected 6 joint values"):
        apply_guards(
            (0.0, 0.0),
            state=make_state(),
            calibration=calibration,
            model=SO101,
            limits=SafetyLimits(),
        )


def test_guards_are_pure(calibration: Calibration) -> None:
    state = make_state()
    limits = SafetyLimits()
    first = apply_guards(
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        state=state,
        calibration=calibration,
        model=SO101,
        limits=limits,
    )
    second = apply_guards(
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        state=state,
        calibration=calibration,
        model=SO101,
        limits=limits,
    )
    assert first == second


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"max_step_deg": 0.0}, "max_step_deg"),
        ({"limit_margin_deg": -1.0}, "limit_margin_deg"),
        ({"temp_warn_c": 80.0, "temp_max_c": 70.0}, "temp_warn_c"),
        ({"voltage_min_v": 0.0}, "voltage_min_v"),
        ({"max_consecutive_failures": 0}, "max_consecutive_failures"),
    ],
)
def test_invalid_limits_are_rejected(kwargs: dict[str, float], match: str) -> None:
    with pytest.raises(ConfigurationError, match=match):
        SafetyLimits(**kwargs)  # type: ignore[arg-type]


def test_step_for_converts_speed_to_a_per_tick_limit() -> None:
    limits = SafetyLimits(max_step_deg=30.0, max_speed_deg_s=720.0)
    assert limits.step_for(50.0) == pytest.approx(14.4)
    assert limits.step_for(200.0) == pytest.approx(3.6)
    # the equivalent physical speed is identical at every rate
    assert limits.step_for(50.0) * 50.0 == pytest.approx(limits.step_for(200.0) * 200.0)


def test_step_for_keeps_the_stricter_of_both_limits() -> None:
    limits = SafetyLimits(max_step_deg=5.0, max_speed_deg_s=400.0)
    assert limits.step_for(50.0) == pytest.approx(5.0)  # speed would allow 8
    # the default has no software limit, matching LeRobot's max_relative_target=None
    assert math.isinf(SafetyLimits().step_for(200.0))
    # a fixed cap is independent of the loop rate
    assert SafetyLimits(max_step_deg=5.0).step_for(200.0) == pytest.approx(5.0)


def test_step_override_tightens_the_guard(calibration: Calibration) -> None:
    report = apply_guards(
        (30.0, *ZERO[1:]),
        state=make_state(),
        calibration=calibration,
        model=SO101,
        limits=SafetyLimits(max_step_deg=5.0),
        step_deg=0.5,
    )
    assert report.goal_deg[0] == pytest.approx(0.5)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"max_speed_deg_s": 0.0}, "max_speed_deg_s"),
        ({"max_speed_deg_s": -10.0}, "max_speed_deg_s"),
    ],
)
def test_invalid_speed_limit_is_rejected(kwargs: dict[str, float], match: str) -> None:
    with pytest.raises(ConfigurationError, match=match):
        SafetyLimits(**kwargs)  # type: ignore[arg-type]


def test_step_for_rejects_a_bad_rate() -> None:
    with pytest.raises(ConfigurationError, match="hz must be positive"):
        SafetyLimits().step_for(0.0)
