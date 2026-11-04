"""ArmBridge: snapshots, commands, staleness guard and torque control."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from pysoarm.arm import SOArm
from pysoarm.bridge import ArmBridge, JointStateFrame
from pysoarm.bus.fake import FakeBus
from pysoarm.exceptions import BusConnectionError, ConfigurationError, SafetyViolation
from pysoarm.models import SO101


class Clock:
    """Controllable monotonic clock."""

    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def bus() -> FakeBus:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


@pytest.fixture
def clock() -> Clock:
    return Clock()


@pytest.fixture
def bridge(bus: FakeBus, calibration_path: Path, clock: Clock) -> ArmBridge:
    arm = SOArm(bus=bus, calibration=calibration_path, clock=clock)
    arm.connect()
    return ArmBridge(arm, clock=clock)


def test_read_produces_a_ros_shaped_frame(bridge: ArmBridge) -> None:
    frame = bridge.read(health=True)
    assert isinstance(frame, JointStateFrame)
    assert frame.names == SO101.joints
    assert len(frame.positions_deg) == SO101.joint_count
    assert len(frame.positions_rad) == SO101.joint_count
    assert frame.positions_rad[0] == pytest.approx(math.radians(frame.positions_deg[0]))
    assert bridge.frame is frame
    assert set(frame.as_dict()) >= {"names", "positions_deg", "temperatures_c"}


def test_command_in_degrees_and_radians(bridge: ArmBridge, bus: FakeBus) -> None:
    bridge.read(health=False)
    sent = bridge.command((0.0,) * SO101.joint_count)
    assert len(sent) == SO101.joint_count
    bridge.command_from_radians((0.0,) * SO101.joint_count)
    assert any(call[0] == "sync_write" for call in bus.calls)


def test_command_requires_fresh_state(bridge: ArmBridge, clock: Clock) -> None:
    with pytest.raises(SafetyViolation, match="no state yet"):
        bridge.command((0.0,) * SO101.joint_count)

    bridge.read(health=False)
    clock.now += 5.0  # older than the default 0.25 s
    with pytest.raises(SafetyViolation, match="refusing to command"):
        bridge.command((0.0,) * SO101.joint_count)


def test_command_rejects_wrong_joint_count(bridge: ArmBridge) -> None:
    bridge.read(health=False)
    with pytest.raises(ConfigurationError, match="Expected 6 joint values"):
        bridge.command((0.0, 0.0))


def test_enable_holds_the_measured_pose(bridge: ArmBridge, bus: FakeBus) -> None:
    bus.disable_torque()
    frame = bridge.enable()
    assert bus.get_register(1, "torque_enable") == 1
    for mid, value in zip(bus.motor_ids, frame.positions_deg, strict=True):
        del value
        assert bus.get_register(mid, "goal_position") == bus.get_register(mid, "present_position")


def test_disable_requires_an_open_bus(bus: FakeBus, calibration_path: Path) -> None:
    arm = SOArm(bus=bus, calibration=calibration_path)
    bridge = ArmBridge(arm)
    with pytest.raises(BusConnectionError):
        bridge.disable()


def test_bridge_rejects_bad_max_age(bus: FakeBus, calibration_path: Path) -> None:
    with pytest.raises(ConfigurationError, match="max_command_age_s"):
        ArmBridge(SOArm(bus=bus, calibration=calibration_path), max_command_age_s=0.0)


def test_names_follow_the_model(bridge: ArmBridge) -> None:
    assert bridge.names == bridge.arm.model.joints == SO101.joints
