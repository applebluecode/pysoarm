"""A small, framework-agnostic bridge for exposing an arm to another process.

ROS 2 nodes, web APIs and test harnesses all need the same three things: a snapshot with
stable field names, a way to push a command, and explicit torque control. :class:`ArmBridge`
provides exactly that on top of :class:`~pysoarm.arm.SOArm`, with **degrees** at the boundary
and one safety rule that matters for remote control: **a command is rejected when the state it
would be applied to is stale**, so a stalled control loop cannot act on an old reading.

The ROS 2 package (``ros2/pysoarm_ros2``) is a thin adapter over this class; keeping the logic
here means it is testable without ROS installed.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from pysoarm.arm import SOArm
from pysoarm.exceptions import ConfigurationError, SafetyViolation
from pysoarm.state import ArmState

__all__ = ["ArmBridge", "JointStateFrame"]

#: Default maximum age of the state a command may be based on.
DEFAULT_MAX_COMMAND_AGE_S = 0.25


@dataclass(frozen=True, slots=True)
class JointStateFrame:
    """Snapshot shaped for publishing (degrees, model joint order).

    Args:
        names: Joint names.
        positions_deg: Joint positions in degrees.
        velocities_deg_s: Indicative joint velocities in deg/s.
        temperatures_c: Motor temperatures in degrees Celsius.
        voltages_v: Supply voltage per motor.
        t: Monotonic timestamp of the read.
    """

    names: tuple[str, ...]
    positions_deg: tuple[float, ...]
    velocities_deg_s: tuple[float, ...]
    temperatures_c: tuple[int, ...]
    voltages_v: tuple[float, ...]
    t: float

    @property
    def positions_rad(self) -> tuple[float, ...]:
        """Positions in radians, for ROS ``sensor_msgs/JointState``."""
        return tuple(math.radians(value) for value in self.positions_deg)

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view."""
        return {
            "names": list(self.names),
            "positions_deg": list(self.positions_deg),
            "velocities_deg_s": list(self.velocities_deg_s),
            "temperatures_c": list(self.temperatures_c),
            "voltages_v": list(self.voltages_v),
            "t": self.t,
        }


class ArmBridge:
    """Command/state bridge around one :class:`~pysoarm.arm.SOArm`.

    Args:
        arm: Connected (or connectable) arm. The bridge never calls ``connect``/``stop``;
            the owner controls the lifetime.
        max_command_age_s: Reject commands when the latest state is older than this.
        clock: Monotonic clock, injectable for tests.

    Raises:
        ConfigurationError: If ``max_command_age_s`` is not positive.
    """

    def __init__(
        self,
        arm: SOArm,
        *,
        max_command_age_s: float = DEFAULT_MAX_COMMAND_AGE_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_command_age_s <= 0:
            raise ConfigurationError("max_command_age_s must be positive")
        self._arm = arm
        self._max_age = max_command_age_s
        self._clock = clock
        self._frame: JointStateFrame | None = None
        self._state: ArmState | None = None

    @property
    def arm(self) -> SOArm:
        """The wrapped arm."""
        return self._arm

    @property
    def names(self) -> tuple[str, ...]:
        """Joint names in the model's order."""
        return self._arm.model.joints

    def read(self, *, health: bool | None = None) -> JointStateFrame:
        """Read the arm and update the cached frame.

        Args:
            health: Forwarded to :meth:`SOArm.read_once` (``None`` = amortised refresh).

        Returns:
            The fresh :class:`JointStateFrame`.
        """
        state = self._arm.read_once(health=health)
        self._state = state
        self._frame = JointStateFrame(
            names=self.names,
            positions_deg=state.q_deg,
            velocities_deg_s=state.dq_deg_s,
            temperatures_c=state.temperature_c,
            voltages_v=state.voltage_v,
            t=state.t,
        )
        return self._frame

    @property
    def frame(self) -> JointStateFrame | None:
        """Most recent frame, or ``None`` before the first read."""
        return self._frame

    def _require_fresh_state(self) -> ArmState:
        if self._state is None:
            raise SafetyViolation("no state yet: call read() before commanding the arm")
        age = self._clock() - self._state.t
        if age > self._max_age:
            raise SafetyViolation(
                f"state is {age:.2f} s old (limit {self._max_age:.2f} s); refusing to command "
                "the arm from a stale snapshot"
            )
        return self._state

    def command(self, positions_deg: Sequence[float]) -> tuple[float, ...]:
        """Send a joint goal in degrees.

        Args:
            positions_deg: Target joint positions, in the model's joint order.

        Returns:
            The goal actually written (after clipping).

        Raises:
            ConfigurationError: Wrong number of joints.
            SafetyViolation: No state yet, or the state is stale.
            CommunicationError: The write failed.
        """
        self._require_fresh_state()
        if len(positions_deg) != self._arm.model.joint_count:
            raise ConfigurationError(
                f"Expected {self._arm.model.joint_count} joint values, got {len(positions_deg)}"
            )
        return self._arm.write_goal(positions_deg)

    def command_from_radians(self, positions_rad: Sequence[float]) -> tuple[float, ...]:
        """Send a joint goal given in radians (convenience for ROS callers)."""
        return self.command(tuple(math.degrees(value) for value in positions_rad))

    def command_from_frame(self, frame: JointStateFrame) -> tuple[float, ...]:
        """Send the positions of ``frame`` (used for ``hold``)."""
        return self.command(frame.positions_deg)

    def enable(self) -> JointStateFrame:
        """Enable torque and hold the measured pose."""
        frame = self.read(health=False)
        self._arm.write_goal(frame.positions_deg)
        self._arm.enable_torque()
        return frame

    def disable(self) -> None:
        """Disable torque (the arm goes limp)."""
        self._arm.disable_torque()
