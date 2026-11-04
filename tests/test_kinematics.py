"""Poses, the Kinematics protocol, and SOArm's Cartesian entry points."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from pysoarm.arm import SOArm
from pysoarm.bus.fake import FakeBus
from pysoarm.exceptions import ConfigurationError, NotReachableError
from pysoarm.kinematics.base import Kinematics, Pose
from pysoarm.models import SO101


# --------------------------------------------------------------------------- #
# Pose
# --------------------------------------------------------------------------- #
def test_pose_round_trips_rpy() -> None:
    pose = Pose.from_xyz_rpy(0.3, -0.1, 0.25, roll=10.0, pitch=-20.0, yaw=45.0)
    assert pose.to_xyz_rpy() == pytest.approx((0.3, -0.1, 0.25, 10.0, -20.0, 45.0), abs=1e-9)


def test_pose_round_trips_in_radians() -> None:
    pose = Pose.from_xyz_rpy(0.0, 0.0, 1.0, yaw=math.pi / 2, degrees=False)
    _, _, _, _, _, yaw_deg = pose.to_xyz_rpy()
    assert yaw_deg == pytest.approx(90.0)


def test_pose_normalises_the_quaternion() -> None:
    pose = Pose(0.0, 0.0, 0.0, qw=2.0, qx=0.0, qy=0.0, qz=0.0)
    assert pose.qw == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"qw": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0}, "zero norm"),
        ({"x": math.nan}, "non-finite"),
        ({"qz": math.inf}, "non-finite"),
    ],
)
def test_pose_rejects_degenerate_input(kwargs: dict[str, float], match: str) -> None:
    base = {"x": 0.0, "y": 0.0, "z": 0.0}
    with pytest.raises(ConfigurationError, match=match):
        Pose(**{**base, **kwargs})


def test_pose_distance_and_angle() -> None:
    origin = Pose(0.0, 0.0, 0.0)
    shifted = Pose(0.3, 0.4, 0.0)
    rotated = Pose.from_xyz_rpy(0.0, 0.0, 0.0, yaw=90.0)
    assert origin.distance_to(shifted) == pytest.approx(0.5)
    assert origin.angle_to(rotated) == pytest.approx(math.pi / 2)
    assert origin.angle_to(origin) == pytest.approx(0.0)


def test_pose_as_dict_is_json_friendly() -> None:
    payload = Pose.from_xyz_rpy(0.1, 0.2, 0.3).as_dict()
    assert payload["xyz_m"] == [0.1, 0.2, 0.3]
    assert payload["rpy_deg"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    quaternion = payload["quaternion_wxyz"]
    assert isinstance(quaternion, list)
    assert len(quaternion) == 4


# --------------------------------------------------------------------------- #
# SOArm integration, with a stub solver
# --------------------------------------------------------------------------- #
class FakeKinematics:
    """Deterministic solver: FK returns the joints, IK returns a fixed configuration."""

    def __init__(self, solution: tuple[float, ...], pose: Pose | None = None) -> None:
        self.solution = solution
        self.pose = pose or Pose.from_xyz_rpy(0.2, 0.0, 0.1)
        self.calls: list[tuple[str, object]] = []

    @property
    def joint_names(self) -> tuple[str, ...]:
        return SO101.joints

    def forward(self, q_deg: object) -> Pose:
        self.calls.append(("forward", q_deg))
        return self.pose

    def inverse(self, pose: Pose, **kwargs: object) -> tuple[float, ...]:
        self.calls.append(("inverse", pose))
        del pose, kwargs
        return self.solution


@pytest.fixture
def bus() -> FakeBus:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


def test_fake_solver_satisfies_the_protocol() -> None:
    assert isinstance(FakeKinematics((0.0,) * 6), Kinematics)


def test_forward_uses_the_measured_state(bus: FakeBus, calibration_path: Path) -> None:
    kinematics = FakeKinematics((0.0,) * 6)
    with SOArm(bus=bus, calibration=calibration_path, kinematics=kinematics) as arm:
        pose = arm.forward()
    assert pose == kinematics.pose
    assert kinematics.calls[0][0] == "forward"
    assert arm.kinematics is kinematics


def test_forward_accepts_explicit_joints(bus: FakeBus, calibration_path: Path) -> None:
    kinematics = FakeKinematics((0.0,) * 6)
    with SOArm(bus=bus, calibration=calibration_path, kinematics=kinematics) as arm:
        arm.forward((0.0,) * 6)
        with pytest.raises(ConfigurationError, match="Expected 6 joint values"):
            arm.forward((0.0,) * 3)


def test_forward_without_kinematics_is_an_error(bus: FakeBus, calibration_path: Path) -> None:
    with (
        SOArm(bus=bus, calibration=calibration_path) as arm,
        pytest.raises(ConfigurationError, match="no kinematics configured"),
    ):
        arm.forward()


def test_move_to_pose_solves_then_moves(bus: FakeBus, calibration_path: Path) -> None:
    target = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    kinematics = FakeKinematics(target)
    with SOArm(bus=bus, calibration=calibration_path, kinematics=kinematics) as arm:
        state = arm.move_to_pose(kinematics.pose, duration=0.03, hz=100)
    assert next(call[0] for call in kinematics.calls) == "inverse"
    assert state.q_deg == pytest.approx(target, abs=0.2)


def test_unreachable_target_leaves_the_arm_untouched(bus: FakeBus, calibration_path: Path) -> None:
    class Missing(FakeKinematics):
        def inverse(self, pose: Pose, **kwargs: object) -> tuple[float, ...]:
            del pose, kwargs
            raise NotReachableError("residual 3.2 m")

    kinematics = Missing((0.0,) * 6)
    with SOArm(bus=bus, calibration=calibration_path, kinematics=kinematics) as arm:
        goals_before = [bus.get_register(mid, "goal_position") for mid in bus.motor_ids]
        with pytest.raises(NotReachableError, match="residual"):
            arm.move_to_pose(kinematics.pose)
        goals_after = [bus.get_register(mid, "goal_position") for mid in bus.motor_ids]
    assert goals_before == goals_after


def test_move_to_pose_rejects_a_wrong_length_solution(bus: FakeBus, calibration_path: Path) -> None:
    kinematics = FakeKinematics((0.0, 0.0))
    with (
        SOArm(bus=bus, calibration=calibration_path, kinematics=kinematics) as arm,
        pytest.raises(ConfigurationError, match="Kinematics returned 2 joints"),
    ):
        arm.move_to_pose(kinematics.pose)
