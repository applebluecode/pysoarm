"""Placo-backed kinematics on a two-link URDF (skipped when the extra is absent)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from pysoarm.exceptions import ConfigurationError, KinematicsError, NotReachableError
from pysoarm.kinematics.base import Pose
from pysoarm.kinematics.placo_solver import PlacoKinematics, require_placo

pytest.importorskip("placo", reason='needs the extra: pip install "pysoarm[kinematics]"')

URDF = Path(__file__).parent / "data" / "two_link.urdf"
JOINTS = ("j1", "j2")


@pytest.fixture(scope="module")
def kinematics() -> PlacoKinematics:
    return PlacoKinematics(URDF, target_frame="tool0", joint_names=JOINTS)


def test_forward_kinematics_matches_the_geometry(kinematics: PlacoKinematics) -> None:
    # both links along +x: 1.0 m + 0.5 m
    assert kinematics.forward((0.0, 0.0)).position == pytest.approx((1.5, 0.0, 0.0), abs=1e-6)
    # first link rotated 90 degrees about z
    assert kinematics.forward((90.0, 0.0)).position == pytest.approx((0.0, 1.5, 0.0), abs=1e-6)
    # elbow folded: 1.0 - 0.5 along x
    assert kinematics.forward((0.0, 180.0)).position == pytest.approx((0.5, 0.0, 0.0), abs=1e-6)


def test_inverse_kinematics_reaches_the_target(kinematics: PlacoKinematics) -> None:
    target = kinematics.forward((17.11, 138.59))
    solution = kinematics.inverse(target, seed_deg=(0.0, 0.0))
    assert len(solution) == 2
    assert kinematics.forward(solution).distance_to(target) < 1e-3


def test_inverse_kinematics_accepts_an_explicit_pose(kinematics: PlacoKinematics) -> None:
    pose = Pose.from_xyz_rpy(0.5, 0.5, 0.0)
    solution = kinematics.inverse(pose, max_iters=400)
    assert kinematics.forward(solution).distance_to(pose) < 1e-3


def test_unreachable_target_raises(kinematics: PlacoKinematics) -> None:
    with pytest.raises(NotReachableError, match="IK did not reach"):
        kinematics.inverse(Pose(5.0, 5.0, 0.0), max_iters=20)


def test_joint_names_are_exposed(kinematics: PlacoKinematics) -> None:
    assert kinematics.joint_names == JOINTS
    assert kinematics.target_frame == "tool0"


def test_wrong_joint_count_is_rejected(kinematics: PlacoKinematics) -> None:
    with pytest.raises(ConfigurationError, match="Expected 2 joint values"):
        kinematics.forward((0.0,))


def test_unknown_joint_and_frame_are_rejected() -> None:
    with pytest.raises(KinematicsError, match="are not in"):
        PlacoKinematics(URDF, joint_names=("j1", "j99"))
    with pytest.raises(KinematicsError, match=r"not in two_link\.urdf"):
        PlacoKinematics(URDF, target_frame="no_such_frame")


def test_missing_urdf_is_reported(tmp_path: Path) -> None:
    with pytest.raises(KinematicsError, match="URDF not found"):
        PlacoKinematics(tmp_path / "nope.urdf")


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"position_gain": 0.0}, "position_gain"),
        ({"orientation_gain": -1.0}, "orientation_gain"),
        ({"max_iters": 0}, "max_iters"),
        ({"tolerance_m": 0.0}, "tolerance_m"),
    ],
)
def test_invalid_solver_options_are_rejected(kwargs: dict[str, float], match: str) -> None:
    with pytest.raises(ConfigurationError, match=match):
        PlacoKinematics(URDF, **kwargs)  # type: ignore[arg-type]


def test_missing_dependency_reports_the_install_hint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def absent() -> object:
        raise ModuleNotFoundError("No module named 'placo'")

    monkeypatch.setattr("pysoarm.kinematics.placo_solver._import_placo", absent)
    with pytest.raises(KinematicsError, match=r"pysoarm\[kinematics\]"):
        require_placo()


def test_orientation_is_ignored_by_default(kinematics: PlacoKinematics) -> None:
    # a 2-DoF planar arm cannot hold an arbitrary orientation; the default must not fight it
    solution = kinematics.inverse(kinematics.forward((0.0, 90.0)), max_iters=100)
    assert math.isfinite(solution[0]) and math.isfinite(solution[1])
