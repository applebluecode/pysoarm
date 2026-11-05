"""A thin, explicit facade over the gripper joint.

The SO-101 gripper is a sixth STS3215 in position mode. It has no force sensor and no
independent controller, so this class deliberately stays a *thin* wrapper around the joint's
calibrated travel and the existing :meth:`~pysoarm.arm.SOArm.write_goal` path::

    arm.gripper.close()          # 0 % of the calibrated travel
    arm.gripper.move(40)         # 40 % open
    arm.gripper.open()           # 100 % open
    arm.gripper.homing()         # record the current closed position as the zero end

``percent == 0`` always means "closed end of the calibrated range" (``range_min``) and
``100`` the "open end" (``range_max``), independent of the sign introduced by ``drive_mode``.
Gripping force is limited by the servo's ``protection_current`` / ``overload_torque``
registers (written at connect time); there is no closed-loop force control.
"""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

from pysoarm.calibration import Calibration, JointCalibration
from pysoarm.exceptions import CalibrationError, ConfigurationError
from pysoarm.state import ArmState

if TYPE_CHECKING:
    from pysoarm.arm import SOArm

__all__ = ["Gripper"]

#: Opening percentages are clamped to this inclusive range.
PERCENT_MAX = 100.0


class Gripper:
    """Percent-of-travel control of the gripper joint.

    Args:
        arm: The arm that owns the gripper. A leader arm is accepted for reading, but
            :meth:`move`/:meth:`homing` refuse to command it.

    Raises:
        ConfigurationError: If the model has no gripper joint.
    """

    def __init__(self, arm: SOArm) -> None:
        self._arm = arm
        joint = arm.model.gripper_joint
        if joint is None:
            raise ConfigurationError(f"model {arm.model.name!r} has no gripper")
        self._joint = joint

    # -- geometry ---------------------------------------------------------- #
    def _calibration(self) -> JointCalibration:
        calibration = self._arm.calibration
        if calibration is None:
            raise CalibrationError("no calibration loaded; call connect() first")
        return calibration.for_joint(self._joint)

    @property
    def joint(self) -> str:
        """Name of the gripper joint."""
        return self._joint

    @property
    def closed_deg(self) -> float:
        """Joint angle at the closed end of the calibrated travel, in degrees."""
        cal = self._calibration()
        return cal.raw_to_deg(cal.range_min)

    @property
    def open_deg(self) -> float:
        """Joint angle at the open end of the calibrated travel, in degrees."""
        cal = self._calibration()
        return cal.raw_to_deg(cal.range_max)

    def percent(self, state: ArmState | None = None) -> float:
        """Current opening in percent (``0`` closed, ``100`` open).

        Args:
            state: Snapshot to use; defaults to the arm's cached state (reading once if
                there is none).

        Returns:
            Opening in ``0..100``, clamped to the calibrated travel.
        """
        if state is None:
            state = self._arm.state if self._arm.state is not None else self._arm.read_once()
        assert state is not None  # narrowed by the expression above
        index = self._arm.model.joints.index(self._joint)
        closed, opened = self.closed_deg, self.open_deg
        if opened == closed:
            return 0.0
        raw = (state.q_deg[index] - closed) / (opened - closed) * 100.0
        return min(100.0, max(0.0, raw))

    def _goal(self, percent: float) -> tuple[float, ...]:
        if not 0.0 <= percent <= PERCENT_MAX:
            raise ConfigurationError(f"gripper percent must be in 0..100, got {percent}")
        closed, opened = self.closed_deg, self.open_deg
        target = closed + (opened - closed) * percent / 100.0
        # Read before commanding: repeated calls must converge, not clip against a stale
        # cached state. The read is one sync transaction, dwarfed by the servo travel.
        state = self._arm.read_once(health=False)
        goal = list(state.q_deg)
        goal[self._arm.model.joints.index(self._joint)] = target
        return tuple(goal)

    # -- motion ------------------------------------------------------------ #
    def move(self, percent: float) -> tuple[float, ...]:
        """Move to an opening percentage.

        Args:
            percent: ``0`` (closed) to ``100`` (open).

        Returns:
            The goal actually written (after the safety guards).

        Raises:
            ConfigurationError: On a read-only arm or an out-of-range percentage.
        """
        if self._arm.is_read_only:
            raise ConfigurationError("leader arms are read-only; the gripper is not moved")
        return self._arm.write_goal(self._goal(percent))

    def open(self) -> tuple[float, ...]:
        """Drive the gripper to the open end of its travel."""
        return self.move(100.0)

    def close(self) -> tuple[float, ...]:
        """Drive the gripper to the closed end of its travel."""
        return self.move(0.0)

    def hold(self) -> ArmState:
        """Command every joint, including the gripper, to its measured position."""
        if self._arm.is_read_only:
            raise ConfigurationError("leader arms are read-only; the gripper is not moved")
        return self._arm.hold()

    def homing(self) -> JointCalibration:
        """Use the current position as the closed end of the gripper travel.

        Intended flow: close the jaws by hand (torque off) on the object-free end stop, then
        call :meth:`homing`. The new range and the existing homing offset are written to the
        motors and become active immediately.

        Returns:
            The updated :class:`~pysoarm.calibration.JointCalibration`.

        Raises:
            CalibrationError: If the current position is beyond the calibrated open end.
            PysoarmError: If a bus transaction fails.
        """
        if self._arm.is_read_only:
            raise ConfigurationError("leader arms are read-only; calibration is unchanged")
        calibration = self._arm.calibration
        if calibration is None:
            raise CalibrationError("no calibration loaded; call connect() first")
        old = calibration.for_joint(self._joint)
        raw = self._arm.raw_position(self._joint)
        if not 0 <= raw < old.range_max:
            raise CalibrationError(
                f"gripper raw position {raw} is not below the open end {old.range_max}; "
                "close the jaws first"
            )
        updated = replace(old, range_min=raw)
        joints = dict(calibration.joints)
        joints[self._joint] = updated
        self._arm.write_calibration(Calibration(joints))
        return updated
