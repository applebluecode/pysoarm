"""``SOArm`` against the fake bus: lifecycle, guards, loops and the watchdog."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import pytest

from pysoarm.arm import LoopStats, SOArm
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
    # the default has no software step cap, so a goal inside the travel is sent verbatim
    requested = (10.0, -10.0, 20.0, 0.0, 90.0, 40.0)
    assert arm.write_goal(requested) == pytest.approx(requested)


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


# --------------------------------------------------------------------------- #
# 0.1.1 additions: roles, safety surface, diagnostics, loop statistics
# --------------------------------------------------------------------------- #
def test_model_number_mismatch_is_rejected(bus: FakeBus, calibration_path: Path) -> None:
    broken = FakeBus(model_number=999)
    for mid in broken.motor_ids:
        broken.set_register(mid, "min_position_limit", 800)
        broken.set_register(mid, "max_position_limit", 3200)
    with pytest.raises(ConfigurationError, match="Unexpected servo model"):
        SOArm(bus=broken).connect()
    tolerated = SOArm(bus=broken)
    tolerated.connect(allow_model_mismatch=True)
    tolerated.stop()


def test_leader_is_read_only_and_stays_limp(bus: FakeBus, calibration_path: Path) -> None:
    arm = SOArm(bus=bus, calibration=calibration_path, role="leader")
    arm.connect()
    try:
        assert arm.role == "leader"
        assert arm.is_read_only
        assert arm.is_enabled() == dict.fromkeys(SO101.joints, False)
        assert all(bus.get_register(mid, "goal_position") == 2048 for mid in bus.motor_ids)
        with pytest.raises(ConfigurationError, match="read-only"):
            arm.write_goal((0.0,) * SO101.joint_count)
        with pytest.raises(ConfigurationError, match="read-only"):
            arm.enable_torque()
        with pytest.raises(ConfigurationError, match="read-only"):
            arm.enable([6])
    finally:
        arm.stop()


def test_invalid_role_is_rejected() -> None:
    with pytest.raises(ConfigurationError, match="role must be"):
        SOArm(bus=FakeBus(), role="observer")  # type: ignore[arg-type]


def test_per_joint_enable_disable_and_is_enabled(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    arm.disable([6])
    assert arm.is_enabled(["gripper"]) == {"gripper": False}
    assert arm.is_enabled(["shoulder_pan"]) == {"shoulder_pan": True}
    arm.enable(["gripper"])
    assert arm.is_enabled(["gripper"]) == {"gripper": True}
    arm.disable([1, 2])
    assert arm.is_enabled([1, 2]) == {"shoulder_pan": False, "shoulder_lift": False}
    with pytest.raises(ConfigurationError, match="not part of model"):
        arm.disable([99])


def test_emergency_stop_keeps_the_connection_alive(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    arm.emergency_stop()
    assert arm.is_connected
    assert bus.is_open
    assert arm.is_enabled() == dict.fromkeys(SO101.joints, False)
    arm.emergency_stop()  # idempotent


def test_reset_faults_restores_the_previous_enable_state(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    arm.disable([6])
    assert arm.reset_faults([6]) == {"gripper": False}
    assert arm.is_enabled(["gripper"]) == {"gripper": False}
    arm.enable(["gripper"])
    assert arm.reset_faults(["gripper"]) == {"gripper": True}
    assert arm.is_enabled(["gripper"]) == {"gripper": True}


def test_health_reports_structured_flags(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    for mid in bus.motor_ids:
        bus.set_register(mid, "present_voltage", 90)  # 9.0 V, below the 10 V limit
    bus.set_register(1, "present_temperature", 60)  # above the 55 C warning
    health = arm.health()
    assert list(health) == list(SO101.joints)
    assert health["shoulder_pan"].flags == frozenset({"voltage", "overheat"})
    assert health["shoulder_lift"].flags == frozenset({"voltage"})
    assert health["gripper"].voltage_v == pytest.approx(9.0)
    assert "flags" in health["gripper"].as_dict()


def test_speed_scale_shrinks_every_step(bus: FakeBus, calibration_path: Path) -> None:
    arm = SOArm(
        bus=bus,
        calibration=calibration_path,
        limits=SafetyLimits(max_step_deg=5.0, max_speed_deg_s=None),
    )
    arm.connect()
    assert arm.set_speed_scale(0.1) == pytest.approx(0.1)
    state = arm.state
    assert state is not None
    sent = arm.write_goal((170.0,) * SO101.joint_count)
    for wanted, actual in zip(state.q_deg, sent, strict=True):
        assert abs(wanted - actual) <= 0.5 + 1e-6
    with pytest.raises(ConfigurationError, match="speed scale"):
        arm.set_speed_scale(1.5)


def test_motor_config_round_trip(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    config = arm.motor_config("gripper")
    assert set(config) >= {"acceleration", "p_coefficient", "protection_current"}
    written = arm.set_motor_config("gripper", acceleration=120, p_coefficient=10)
    assert written == {"acceleration": 120, "p_coefficient": 10}
    assert bus.get_register(6, "acceleration") == 120
    with pytest.raises(ConfigurationError, match="Unknown motor config"):
        arm.set_motor_config("gripper", no_such_register=1)


def test_loop_reports_latency_and_serialises(arm: SOArm) -> None:
    arm.connect()

    def keep(state: ArmState, elapsed: float) -> tuple[float, ...]:
        del elapsed
        return state.q_deg

    stats = arm.run_control_loop(keep, hz=200.0, duration=0.03)
    assert stats.ticks >= 1
    assert stats.mean_latency_s > 0.0
    assert stats.p95_latency_s >= stats.mean_latency_s
    assert stats.max_latency_s >= stats.p95_latency_s
    assert stats.deadline_misses == stats.overruns
    payload = stats.as_dict()
    assert set(payload) >= {"p95_latency_s", "deadline_misses", "errors"}


def test_loop_can_reuse_a_cached_state(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    bus.calls.clear()
    stats = arm.run_control_loop(lambda s, t: None, hz=200.0, duration=0.02, read_state_every=0)
    assert stats.ticks >= 1
    assert not [call for call in bus.calls if call[0] == "sync_read"]


def test_loop_can_read_every_n_ticks(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    bus.calls.clear()
    stats = arm.run_control_loop(lambda s, t: None, hz=200.0, duration=0.03, read_state_every=2)
    reads = [call for call in bus.calls if call[0] == "sync_read"]
    assert 0 < len(reads) < stats.ticks


def test_loop_error_policies(arm: SOArm) -> None:
    arm.connect()
    calls = {"n": 0}

    def flaky(state: ArmState, elapsed: float) -> None:
        del state, elapsed
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("policy glitch")

    held = arm.run_control_loop(flaky, hz=200.0, duration=0.03, on_error="hold")
    assert held.errors == 1
    assert held.consecutive_errors_max == 1

    def always_fails(state: ArmState, elapsed: float) -> None:
        del state, elapsed
        raise RuntimeError("policy glitch")

    with pytest.raises(RuntimeError, match="policy glitch"):
        arm.run_control_loop(always_fails, hz=200.0, duration=1.0, on_error="skip")
    with pytest.raises(RuntimeError, match="policy glitch"):
        arm.run_control_loop(always_fails, hz=200.0, duration=1.0)  # abort is the default
    with pytest.raises(ConfigurationError, match="unknown on_error"):
        arm.run_control_loop(flaky, on_error="explode")  # type: ignore[arg-type]


def test_gripper_facade_moves_and_homes(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    gripper = arm.gripper
    assert gripper.joint == "gripper"

    # the default has no software step cap, so one command reaches the end stop
    gripper.close()
    state = arm.read_once(health=False)
    assert gripper.percent(state) == pytest.approx(0.0, abs=1.0)

    gripper.open(wait=True)
    state = arm.read_once(health=False)
    assert gripper.percent(state) == pytest.approx(100.0, abs=1.0)

    gripper.move(40, wait=True)
    state = arm.read_once(health=False)
    assert gripper.percent(state) == pytest.approx(40.0, abs=2.0)

    with pytest.raises(ConfigurationError, match="percent"):
        gripper.move(101.0)

    bus.set_register(6, "present_position", 2048)
    updated = gripper.homing()
    assert updated.range_min == 2048
    assert arm.calibration is not None
    assert arm.calibration.for_joint("gripper").range_min == 2048
    assert bus.get_register(6, "min_position_limit") == 2048


def test_leader_gripper_cannot_move(bus: FakeBus, calibration_path: Path) -> None:
    arm = SOArm(bus=bus, calibration=calibration_path, role="leader")
    arm.connect()
    try:
        with pytest.raises(ConfigurationError, match="read-only"):
            arm.gripper.close()
        with pytest.raises(ConfigurationError, match="read-only"):
            arm.gripper.homing()
    finally:
        arm.stop()


class StatusBus(FakeBus):
    """Fake bus that fails one read with a servo alarm, to check flag latching."""

    def __init__(self) -> None:
        super().__init__()
        self.inject_status: int | None = None

    def sync_read(
        self, motor_ids: Sequence[int], register: str, *, retries: int | None = None
    ) -> dict[int, int]:
        if self.inject_status is not None and register == "present_position":
            status, self.inject_status = self.inject_status, None
            raise CommunicationError(
                "injected servo alarm",
                motor_id=3,
                register=register,
                status=status,
                transient=True,
            )
        return super().sync_read(motor_ids, register, retries=retries)


def test_status_alarm_bits_are_latched_into_health(
    calibration_path: Path,
) -> None:
    bus = StatusBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    arm = SOArm(bus=bus, calibration=calibration_path)
    arm.connect()
    try:
        bus.inject_status = 0x08
        with pytest.raises(CommunicationError, match="servo alarm"):
            arm.read_once(health=False)
        health = arm.health()
        assert "overcurrent" in health["elbow_flex"].flags
        assert health["elbow_flex"].motor_id == 3
        assert "overcurrent" not in health["gripper"].flags
    finally:
        arm.stop()


def test_read_once_can_override_the_health_period(arm: SOArm, bus: FakeBus) -> None:
    arm.connect()
    arm._health_period_s = 100.0
    bus.calls.clear()
    arm.read_once()
    assert [call[0] for call in bus.calls] == ["sync_read"]
    bus.calls.clear()
    arm.read_once(health_period_s=0.0)
    assert len(bus.calls) == 2  # position + one incremental health register


def test_latency_percentile_is_bounded() -> None:
    empty = LoopStats(ticks=0, overruns=0, elapsed_s=0.0)
    assert empty.p95_latency_s == 0.0
    assert empty.latency_percentile(0.5) == 0.0
    with pytest.raises(ConfigurationError, match="fraction"):
        empty.latency_percentile(1.5)
