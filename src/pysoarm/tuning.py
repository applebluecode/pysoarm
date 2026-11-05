"""Read and write the STS3215 tuning registers exposed by ``SOArm``.

The registers are configuration, not control: PID gains, acceleration, torque/current limits
and the servo's own speed limit. This module keeps the list of supported registers and the
read-modify-write plumbing in one place; policy (read-only leader arms) stays on
:class:`~pysoarm.arm.SOArm`.

Three different "speed" knobs exist on purpose:

* ``SafetyLimits.step_for(hz)`` — software limit, the only one that protects every backend;
* ``goal_speed`` — the servo's internal velocity ceiling (``0`` means unlimited);
* ``acceleration`` / ``maximum_acceleration`` — the servo's ramp, written at connect with
  LeRobot's maximum values.

``goal_speed`` defaults to ``0`` (unlimited) so the LeRobot-compatible configuration is
unchanged; set it through :meth:`SOArm.set_motor_config` when the servo should limit itself.
"""

from __future__ import annotations

from pysoarm.bus.base import Bus
from pysoarm.exceptions import ConfigurationError
from pysoarm.models.base import RobotModel

__all__ = ["MOTOR_CONFIG_REGISTERS", "read_motor_config", "write_motor_config"]

#: Registers returned by ``SOArm.motor_config`` and accepted by ``SOArm.set_motor_config``.
MOTOR_CONFIG_REGISTERS: tuple[str, ...] = (
    "acceleration",
    "maximum_acceleration",
    "goal_speed",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "torque_limit",
    "protection_current",
    "overload_torque",
)


def read_motor_config(bus: Bus, model: RobotModel, joint: str) -> dict[str, int]:
    """Read every supported tuning register of one joint.

    Args:
        bus: Open transport.
        model: Arm model that maps joint names to motor ids.
        joint: Joint name.

    Returns:
        Mapping ``register -> value`` for :data:`MOTOR_CONFIG_REGISTERS`.
    """
    mid = model.id_of(joint)
    ids = model.motor_ids
    return {register: bus.sync_read(ids, register)[mid] for register in MOTOR_CONFIG_REGISTERS}


def write_motor_config(
    bus: Bus,
    model: RobotModel,
    joint: str,
    values: dict[str, int],
) -> dict[str, int]:
    """Write tuning registers of one joint.

    Args:
        bus: Open transport.
        model: Arm model that maps joint names to motor ids.
        joint: Joint name.
        values: Register/value pairs, each in :data:`MOTOR_CONFIG_REGISTERS`.

    Returns:
        The values written.

    Raises:
        ConfigurationError: On an unknown register name.
    """
    unknown = sorted(set(values) - set(MOTOR_CONFIG_REGISTERS))
    if unknown:
        raise ConfigurationError(
            f"Unknown motor config register(s) {unknown}; known: {list(MOTOR_CONFIG_REGISTERS)}"
        )
    mid = model.id_of(joint)
    for register, value in values.items():
        bus.write(mid, register, int(value))
    return {register: int(value) for register, value in values.items()}
