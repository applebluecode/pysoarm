"""Poses and the kinematics protocol.

A :class:`Pose` is a position in metres plus a unit quaternion — no numpy, no dependency — so
poses can be constructed, compared and logged even when the optional solver is not installed.
The solver lives behind :class:`Kinematics`, which :class:`~pysoarm.arm.SOArm` accepts as an
optional collaborator.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from pysoarm.exceptions import ConfigurationError

__all__ = ["Kinematics", "Pose"]

#: Below this quaternion norm a pose is considered degenerate.
_MIN_QUAT_NORM = 1e-9
#: Tolerance used to decide whether a quaternion is already normalised.
_UNIT_TOLERANCE = 1e-12


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Quaternion ``(w, x, y, z)`` from intrinsic Z-Y-X Euler angles (radians)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rpy_from_quat(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Intrinsic Z-Y-X Euler angles (radians) from a unit quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass(frozen=True, slots=True)
class Pose:
    """End-effector pose: metres plus a unit quaternion ``(w, x, y, z)``.

    Args:
        x: Position along x, metres.
        y: Position along y, metres.
        z: Position along z, metres.
        qw: Quaternion w (scalar) component.
        qx: Quaternion x component.
        qy: Quaternion y component.
        qz: Quaternion z component.

    Raises:
        ConfigurationError: If a value is not finite or the quaternion has zero norm.

    The quaternion is normalised on construction, so ``Pose(...)`` always holds a unit
    rotation and equality is well defined.
    """

    x: float
    y: float
    z: float
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    def __post_init__(self) -> None:
        values = (self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)
        if not all(math.isfinite(value) for value in values):
            raise ConfigurationError(f"Pose contains a non-finite value: {values}")
        norm = math.sqrt(self.qw**2 + self.qx**2 + self.qy**2 + self.qz**2)
        if norm < _MIN_QUAT_NORM:
            raise ConfigurationError("Pose quaternion has zero norm")
        if abs(norm - 1.0) > _UNIT_TOLERANCE:
            object.__setattr__(self, "qw", self.qw / norm)
            object.__setattr__(self, "qx", self.qx / norm)
            object.__setattr__(self, "qy", self.qy / norm)
            object.__setattr__(self, "qz", self.qz / norm)

    @property
    def position(self) -> tuple[float, float, float]:
        """Cartesian position in metres."""
        return (self.x, self.y, self.z)

    @classmethod
    def from_xyz_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        *,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        degrees: bool = True,
    ) -> Pose:
        """Build a pose from position and Euler angles (degrees by default)."""
        if degrees:
            roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
        qw, qx, qy, qz = _quat_from_rpy(roll, pitch, yaw)
        return cls(x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz)

    def to_xyz_rpy(
        self, *, degrees: bool = True
    ) -> tuple[float, float, float, float, float, float]:
        """Position and Euler angles (degrees by default)."""
        roll, pitch, yaw = _rpy_from_quat(self.qw, self.qx, self.qy, self.qz)
        if degrees:
            roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        return (self.x, self.y, self.z, roll, pitch, yaw)

    def distance_to(self, other: Pose) -> float:
        """Euclidean distance between the two positions, in metres."""
        return math.dist(self.position, other.position)

    def angle_to(self, other: Pose) -> float:
        """Angle between the two orientations, in radians (``0`` for identical rotations)."""
        dot = abs(self.qw * other.qw + self.qx * other.qx + self.qy * other.qy + self.qz * other.qz)
        return 2.0 * math.acos(min(1.0, dot))

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view (metres and degrees)."""
        x, y, z, roll, pitch, yaw = self.to_xyz_rpy()
        return {
            "xyz_m": [x, y, z],
            "rpy_deg": [roll, pitch, yaw],
            "quaternion_wxyz": [self.qw, self.qx, self.qy, self.qz],
        }


@runtime_checkable
class Kinematics(Protocol):
    """Forward/inverse kinematics for one arm.

    Implementations are pure functions of their inputs (a solver may cache internally), take
    degrees and metres, and raise :class:`~pysoarm.exceptions.NotReachableError` instead of
    returning a silently saturated joint vector.
    """

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Joint names, in the order of the arrays accepted and returned."""
        ...

    def forward(self, q_deg: Sequence[float]) -> Pose:
        """End-effector pose for a joint configuration, in degrees."""
        ...

    def inverse(
        self,
        pose: Pose,
        *,
        seed_deg: Sequence[float] | None = None,
        max_iters: int | None = None,
        tolerance_m: float | None = None,
    ) -> tuple[float, ...]:
        """Joint configuration (degrees) reaching ``pose``.

        Raises:
            NotReachableError: If the solver did not converge.
        """
        ...
