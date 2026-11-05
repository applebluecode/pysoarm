"""Acceptance tests for the 0.1.2 improvements (``PYSOARM_IMPROVEMENTS.md`` §11).

One group per item L1-L9. Everything runs offline: ``FakeBus`` for the arm-level items, a
small stub bus for the ``doctor --stress`` classification, and injected clocks for timing.
"""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Sequence
from pathlib import Path

import pytest

from pysoarm import SO101, FakeBus, SafetyLimits, SOArm
from pysoarm.calibration import Calibration
from pysoarm.cli import factories
from pysoarm.cli.commands import diagnostics
from pysoarm.cli.main import main
from pysoarm.constants import DEFAULT_MAX_SPEED_DEG_S
from pysoarm.exceptions import CommunicationError, SafetyViolation
from pysoarm.safety import apply_guards
from pysoarm.state import ArmState


def calibrated_bus(**kwargs: object) -> FakeBus:
    bus = FakeBus(**kwargs)  # type: ignore[arg-type]
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


def connected_arm(*, role: str = "follower", **kwargs: object) -> tuple[SOArm, FakeBus]:
    bus = calibrated_bus(**kwargs)
    arm = SOArm(bus=bus, role=role)  # type: ignore[arg-type]
    arm.connect(backoff_s=0)
    return arm, bus


# --------------------------------------------------------------------------- #
# L1  no software speed limit by default (LeRobot parity), opt-in presets
# --------------------------------------------------------------------------- #
def test_l1_default_has_no_speed_or_step_limit() -> None:
    limits = SafetyLimits()
    assert limits.max_step_deg is None
    assert limits.max_speed_deg_s is None
    for hz in (50.0, 100.0, 200.0, 400.0):
        assert math.isinf(limits.step_for(hz))


def test_l1_optional_speed_preset_is_rate_independent() -> None:
    limits = SafetyLimits(max_speed_deg_s=DEFAULT_MAX_SPEED_DEG_S)
    for hz in (50.0, 100.0, 200.0, 400.0):
        assert limits.step_for(hz) * hz == pytest.approx(DEFAULT_MAX_SPEED_DEG_S)


def test_l1_absolute_step_cap_still_wins_when_stricter() -> None:
    limits = SafetyLimits(max_step_deg=5.0, max_speed_deg_s=DEFAULT_MAX_SPEED_DEG_S)
    assert limits.step_for(50.0) == pytest.approx(5.0)  # speed would allow 14.4
    assert limits.step_for(200.0) == pytest.approx(3.6)


def test_l1_configured_step_cap_clips_write_goal(calibration_path: Path) -> None:
    bus = calibrated_bus()
    arm = SOArm(
        bus=bus,
        calibration=calibration_path,
        limits=SafetyLimits(max_step_deg=5.0, max_speed_deg_s=None),
    )
    arm.connect(backoff_s=0)
    try:
        start = arm.read_once(health=False).q_deg
        goal = list(start)
        goal[0] += 40.0
        sent = arm.write_goal(goal)
        assert sent[0] - start[0] == pytest.approx(5.0)
    finally:
        arm.stop()


def test_l1_connect_reports_no_limit_or_the_configured_one(
    caplog: pytest.LogCaptureFixture, calibration_path: Path
) -> None:
    arm = SOArm(bus=calibrated_bus(), calibration=calibration_path)
    with caplog.at_level(logging.INFO, logger="pysoarm.arm"):
        arm.connect(backoff_s=0)
    try:
        assert any("no software speed/step cap" in record.message for record in caplog.records)
    finally:
        arm.stop()

    caplog.clear()
    arm = SOArm(
        bus=calibrated_bus(),
        calibration=calibration_path,
        limits=SafetyLimits(max_speed_deg_s=DEFAULT_MAX_SPEED_DEG_S),
    )
    with caplog.at_level(logging.INFO, logger="pysoarm.arm"):
        arm.connect(backoff_s=0)
    try:
        assert any("720" in record.message for record in caplog.records)
    finally:
        arm.stop()


# --------------------------------------------------------------------------- #
# L2  doctor --stress separates timeouts from status alarms
# --------------------------------------------------------------------------- #
class StubBus:
    """Minimal bus: motor 1 answers a voltage alarm, motor 2 never answers."""

    backend = "stub"

    def __init__(self) -> None:
        self.now = 0.0

    def clock(self) -> float:
        return self.now

    def sync_read(
        self, motor_ids: Sequence[int], register: str, *, retries: int | None = None
    ) -> dict[int, int]:
        del retries
        value = 119 if register == "present_voltage" else 30
        return dict.fromkeys(motor_ids, value)

    def read(self, motor_id: int, register: str, *, retries: int | None = None) -> int:
        del register, retries
        self.now += 0.001
        if motor_id == 1:
            raise CommunicationError("alarm", motor_id=1, status=0x01)
        if motor_id == 2:
            raise CommunicationError("timeout", motor_id=2, status=None)
        return 2048


def test_l2_stress_counts_timeouts_and_status_alarms_separately() -> None:
    bus = StubBus()
    result = diagnostics._stress(
        bus,  # type: ignore[arg-type]
        SO101,
        seconds=0.001,
        min_voltage=5.0,
        clock=bus.clock,
    )
    assert result.timeouts[1] == 0
    assert result.timeouts[2] == 1
    assert result.status_alarms[1] == {"voltage": 1}
    assert result.status_alarms[2] == {}
    assert result.alarm_counts == {"voltage": 1}
    assert result.latency["mean_ms"] > 0.0
    assert result.failed_latency["mean_ms"] > 0.0
    payload = result.as_dict()
    timeouts = payload["timeouts"]
    alarms = payload["status_alarms"]
    assert isinstance(timeouts, dict) and timeouts["2"] == 1
    assert isinstance(alarms, dict) and alarms["1"] == {"voltage": 1}


# --------------------------------------------------------------------------- #
# L3  role-aware supply-voltage threshold
# --------------------------------------------------------------------------- #
def test_l3_leader_defaults_to_the_five_volt_threshold() -> None:
    assert SafetyLimits.for_role("leader").voltage_min_v == pytest.approx(4.0)
    assert SafetyLimits.for_role("follower").voltage_min_v == pytest.approx(10.0)


def test_l3_leader_at_4_9_v_has_no_voltage_flag() -> None:
    arm, _ = connected_arm(role="leader", voltage_v=4.9)
    try:
        health = arm.health()
        assert all("voltage" not in entry.flags for entry in health.values())
    finally:
        arm.stop()


def test_l3_follower_still_refuses_an_undervoltage_goal(calibration_path: Path) -> None:
    arm = SOArm(bus=calibrated_bus(voltage_v=9.0), calibration=calibration_path)
    with pytest.raises(SafetyViolation, match=r"below the 10\.0 V"):
        arm.connect(backoff_s=0)
    arm.stop()


# --------------------------------------------------------------------------- #
# L4  gripper wait semantics
# --------------------------------------------------------------------------- #
def test_l4_gripper_close_wait_reaches_the_end_stop(calibration_path: Path) -> None:
    arm = SOArm(bus=calibrated_bus(), calibration=calibration_path)
    arm.connect(backoff_s=0)
    try:
        gripper = arm.gripper
        gripper.open(wait=True)
        assert gripper.percent() == pytest.approx(100.0, abs=1.0)
        gripper.close(wait=True)
        assert gripper.percent() == pytest.approx(0.0, abs=1.0)
        # without a configured limit, one non-blocking command goes straight to the target
        gripper.open(wait=True)
        gripper.close()
        state = arm.read_once(health=False)
        assert gripper.percent(state) == pytest.approx(0.0, abs=1.0)
    finally:
        arm.stop()


def test_l4_non_blocking_is_clipped_when_a_limit_is_configured(
    calibration_path: Path,
) -> None:
    arm = SOArm(
        bus=calibrated_bus(),
        calibration=calibration_path,
        limits=SafetyLimits(max_step_deg=5.0, max_speed_deg_s=None),
    )
    arm.connect(backoff_s=0)
    try:
        gripper = arm.gripper
        gripper.open(wait=True)
        gripper.close()
        state = arm.read_once(health=False)
        assert gripper.percent(state) > 90.0  # only one 5 deg step from fully open
    finally:
        arm.stop()


# --------------------------------------------------------------------------- #
# L5  goal_speed exposed through motor_config
# --------------------------------------------------------------------------- #
def test_l5_goal_speed_is_readable_and_writable() -> None:
    arm, bus = connected_arm()
    try:
        config = arm.motor_config("shoulder_pan")
        assert config["goal_speed"] == 0
        written = arm.set_motor_config("shoulder_pan", goal_speed=1000)
        assert written == {"goal_speed": 1000}
        assert arm.motor_config("shoulder_pan")["goal_speed"] == 1000
        assert bus.get_register(1, "goal_speed") == 1000
    finally:
        arm.stop()


# --------------------------------------------------------------------------- #
# L6  connect retries transient failures with a backoff
# --------------------------------------------------------------------------- #
def test_l6_connect_survives_transient_read_failures(calibration_path: Path) -> None:
    bus = calibrated_bus()
    bus.fail_next_reads = 2
    arm = SOArm(bus=bus, calibration=calibration_path)
    arm.connect(retries=2, backoff_s=0)
    try:
        assert arm.is_connected
    finally:
        arm.stop()


def test_l6_connect_gives_up_with_an_actionable_message(calibration_path: Path) -> None:
    bus = calibrated_bus()
    bus.fail_next_reads = 50
    arm = SOArm(bus=bus, calibration=calibration_path)
    with pytest.raises(CommunicationError, match="pysoarm doctor") as excinfo:
        arm.connect(retries=1, backoff_s=0)
    assert "gave up after 2 connect attempt(s)" in str(excinfo.value)
    assert excinfo.value.attempts == 2
    assert not arm.is_connected
    assert not bus.is_open


# --------------------------------------------------------------------------- #
# L7  loop statistics attribute the failure to its source
# --------------------------------------------------------------------------- #
def test_l7_loop_distinguishes_read_and_callback_errors() -> None:
    failures = {"leader": 2}

    def callback(state: ArmState, elapsed: float) -> None:
        del state, elapsed
        if failures["leader"] > 0:
            failures["leader"] -= 1
            raise CommunicationError("leader did not answer")

    arm, _ = connected_arm()
    try:
        stats = arm.run_control_loop(callback, hz=200.0, duration=0.05, on_error="hold")
    finally:
        arm.stop()
    assert stats.callback_errors >= 1
    assert stats.read_errors == 0
    assert stats.errors == stats.callback_errors + stats.read_errors + stats.write_errors
    assert set(stats.as_dict()) >= {"read_errors", "callback_errors", "write_errors"}


def test_l7_loop_counts_follower_read_errors_separately() -> None:
    arm, bus = connected_arm()
    bus.fail_next_reads = 1
    try:
        stats = arm.run_control_loop(lambda s, t: None, hz=100.0, duration=0.05, on_error="hold")
    finally:
        arm.stop()
    assert stats.read_errors == 1
    assert stats.callback_errors == 0


# --------------------------------------------------------------------------- #
# L8  min_step_deg suppresses sub-deadband commands
# --------------------------------------------------------------------------- #
def _zero_state() -> ArmState:
    return ArmState(
        q_deg=(0.0,) * SO101.joint_count,
        speed_raw=(0,) * SO101.joint_count,
        current=(0,) * SO101.joint_count,
        temperature_c=(35,) * SO101.joint_count,
        voltage_v=(12.0,) * SO101.joint_count,
        t=0.0,
    )


def test_l8_min_step_skips_a_command_inside_the_deadband(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    limits = SafetyLimits(
        max_step_deg=100.0, max_speed_deg_s=None, min_step_deg=1.0, limit_margin_deg=0.0
    )
    report = apply_guards(
        (0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
        state=_zero_state(),
        calibration=calibration,
        model=SO101,
        limits=limits,
        step_deg=5.0,
    )
    assert report.goal_deg[0] == pytest.approx(0.0)
    assert report.clipped
    assert any("skipped shoulder_pan" in reason for reason in report.reasons)


def test_l8_min_step_default_does_not_change_the_goal(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    report = apply_guards(
        (0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
        state=_zero_state(),
        calibration=calibration,
        model=SO101,
        limits=SafetyLimits(max_step_deg=100.0, max_speed_deg_s=None),
        step_deg=5.0,
    )
    assert report.goal_deg[0] == pytest.approx(0.5)


# --------------------------------------------------------------------------- #
# L9  doctor --json
# --------------------------------------------------------------------------- #
def test_l9_doctor_json_is_machine_readable(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_bus", lambda args: calibrated_bus())
    monkeypatch.setattr(diagnostics, "_service_active", lambda name: False)
    monkeypatch.setattr(diagnostics, "_port_holders", lambda path: [])
    code = main(["doctor", "--port", "/dev/null", "--stress", "0.01", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert set(payload) >= {
        "backend",
        "checks",
        "timeouts",
        "status_alarms",
        "latency",
        "min_voltage_v",
    }
    assert payload["backend"] == "fake"
    assert payload["min_voltage_v"] == pytest.approx(10.0)
    assert all({"name", "ok", "detail", "fatal", "status"} <= set(c) for c in payload["checks"])


def test_l9_doctor_json_uses_the_leader_voltage_threshold(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_bus", lambda args: calibrated_bus(voltage_v=4.9))
    monkeypatch.setattr(diagnostics, "_service_active", lambda name: False)
    monkeypatch.setattr(diagnostics, "_port_holders", lambda path: [])
    code = main(["doctor", "--port", "/dev/null", "--role", "leader", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["min_voltage_v"] == pytest.approx(4.0)
    assert payload["warnings"] == []


def test_l9_stress_result_is_json_serialisable() -> None:
    result = diagnostics.StressResult(
        warnings=("motor 1: 4.7 V",),
        timeouts={1: 0, 2: 3},
        status_alarms={1: {"voltage": 2}},
        latency={"mean_ms": 0.3},
        failed_latency={"mean_ms": 0.1},
    )
    text = json.dumps(result.as_dict())
    assert json.loads(text)["alarm_counts"] == {"voltage": 2}
