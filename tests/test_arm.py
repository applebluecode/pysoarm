"""``SOArm`` against the fake bus: lifecycle, guards, loops and the watchdog."""

from __future__ import annotations

from pathlib import Path

import pytest

from pysoarm.arm import SOArm
from pysoarm.bus.fake import FakeBus
from pysoarm.exceptions import (
    BusConnectionError,
    CalibrationError,
    CommunicationError,
    ConfigurationError,
    WatchdogTimeout,
)
from pysoarm.models import SO101
from pysoarm.safety import SafetyLimits
from pysoarm.state import ArmState


@pytest.fixture
def bus() -> FakeBus:
    """Six servos with a plausible travel, torque on."""
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


@pytest.fixture
def arm(bus: FakeBus, calibration_path: Path) -> SOArm:
    return SOArm(bus=bus, calibration=calibration_path)


def test_connect_reads_calibration_from_the_motors(bus: FakeBus) -> None:
    with SOArm(bus=bus) as arm:
        assert arm.calibration is not None
        assert sorted(arm.calibration.joints) == sorted(SO101.joints)
        assert arm.state is not None
        assert len(arm.state.q_deg) == SO101.joint_count
        assert bus.get_register(1, "torque_enable") == 1


def test_connect_holds_the_measured_pose(bus: FakeBus, calibration_path: Path) -> None:
    with SOArm(bus=bus, calibration=calibration_path):
        for mid in bus.motor_ids:
            assert bus.get_register(mid, "goal_position") == 2048


def test_connect_reports_missing_motors() -> None:
    bus = FakeBus(motor_ids=(1, 2, 3))
    with pytest.raises(BusConnectionError, match="did not answer"):
        SOArm(bus=bus).connect()
    assert not bus.is_open  # the failed connect released the bus


def test_connect_rejects_uncalibrated_motors() -> None:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 2048)
        bus.set_register(mid, "max_position_limit", 2048)
    with pytest.raises(CalibrationError, match="not calibrated"):
        SOArm(bus=bus).connect()


def test_write_goal_returns_what_it_actually_sent(arm: SOArm) -> None:
    arm.connect()
    state = arm.state
    assert state is not None
    sent = arm.write_goal((170.0,) * SO101.joint_count)
    for wanted, actual in zip(state.q_deg, sent, strict=True):
        assert abs(wanted - actual) <= arm.limits.max_step_deg + 1e-6


def test_write_goal_rejects_wrong_joint_count(arm: SOArm) -> None:
    arm.connect()
    with pytest.raises(ConfigurationError, match="Expected 6 joint values"):
        arm.write_goal((0.0, 0.0))


def test_health_registers_are_cached_between_ticks(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    bus.calls.clear()
    arm.read_once(health=False)
    assert [call[0] for call in bus.calls] == ["sync_read"]  # hot path: one transaction

    bus.calls.clear()
    arm.read_once(health=True)
    assert [call[0] for call in bus.calls] == ["sync_read"] * 5  # + 4 health registers


def test_move_to_reaches_the_target(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    target = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    bus.calls.clear()
    state = arm.move_to(target, duration=0.05, hz=100)
    assert state.q_deg == pytest.approx(target, abs=0.2)
    assert any(call[0] == "sync_write" and call[1] == "goal_position" for call in bus.calls)


def test_move_to_rejects_unknown_profile(arm: SOArm) -> None:
    arm.connect()
    with pytest.raises(ConfigurationError, match="Unknown profile"):
        arm.move_to((0.0,) * 6, duration=0.1, profile="cubic")


def test_run_control_loop_paces_and_counts(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    bus.calls.clear()

    def hold_callback(state: ArmState, elapsed: float) -> None:
        del state, elapsed

    stats = arm.run_control_loop(hold_callback, hz=100.0, duration=0.05)
    assert stats.ticks >= 2
    assert stats.mean_hz > 0
    assert stats.overrun_ratio == 0.0
    assert not [call for call in bus.calls if call[0] == "sync_write"]


def test_run_control_loop_writes_callback_goals(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    bus.calls.clear()

    def keep_callback(state: ArmState, elapsed: float) -> tuple[float, ...]:
        del elapsed
        return state.q_deg

    stats = arm.run_control_loop(keep_callback, hz=100.0, duration=0.03)
    assert stats.ticks >= 1
    assert [call for call in bus.calls if call[0] == "sync_write"]


def test_run_control_loop_rejects_bad_rate(arm: SOArm) -> None:
    arm.connect()
    with pytest.raises(ConfigurationError, match="hz must be positive"):
        arm.run_control_loop(lambda s, t: None, hz=0.0)


def test_watchdog_disables_torque_after_repeated_failures(bus: FakeBus) -> None:
    arm = SOArm(bus=bus, limits=SafetyLimits(max_consecutive_failures=2))
    arm.connect()
    bus.fail_next_reads = 5
    with pytest.raises(CommunicationError):
        arm.read_once(health=False)
    with pytest.raises(WatchdogTimeout, match="consecutive bus failures"):
        arm.read_once(health=False)


def test_watchdog_releases_torque(bus: FakeBus) -> None:
    arm = SOArm(bus=bus, limits=SafetyLimits(max_consecutive_failures=1))
    arm.connect()
    bus.fail_next_reads = 1
    with pytest.raises(WatchdogTimeout):
        arm.read_once(health=False)
    assert bus.get_register(1, "torque_enable") == 0


def test_stop_is_idempotent_and_releases_torque(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    arm.stop()
    assert bus.get_register(1, "torque_enable") == 0
    assert not arm.is_connected
    arm.stop()  # must not raise


def test_context_manager_stops_on_exit(bus: FakeBus, calibration_path: Path) -> None:
    with SOArm(bus=bus, calibration=calibration_path) as arm:
        was_connected = arm.is_connected
    # asserted through a local: mypy would otherwise narrow `arm.is_connected` for the rest
    # of the function and mark the post-exit checks as unreachable.
    assert was_connected
    assert not arm.is_connected
    assert bus.get_register(1, "torque_enable") == 0


def test_constructor_requires_a_port_or_a_bus() -> None:
    with pytest.raises(ConfigurationError, match="port= or bus="):
        SOArm()


def test_state_reports_freshness(arm: SOArm) -> None:
    arm.connect()
    state = arm.state
    assert state is not None
    assert not state.is_stale(state.t)
    assert state.is_stale(state.t + 10.0)
    assert set(state.as_dict()) >= {"q_deg", "temperature_c", "voltage_v"}


def test_health_refresh_is_spread_over_ticks(bus: FakeBus, calibration_path: Path) -> None:
    arm = SOArm(bus=bus, calibration=calibration_path, health_period_s=0.0)
    arm.connect()
    bus.calls.clear()
    for _ in range(4):
        arm.read_once()  # health=None, stale cache -> one extra register per tick
    reads = [call for call in bus.calls if call[0] == "sync_read"]
    registers = [call[1] for call in reads]
    assert registers.count("present_position") == 4
    # one health register per tick, cycling through all four
    assert registers.count("present_temperature") == 1
    assert registers.count("present_voltage") == 1
    assert len(reads) == 8
