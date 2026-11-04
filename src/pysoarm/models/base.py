"""``RobotModel``: the static description of one arm variant.

A model fixes the joint order, the motor id behind each joint and the servo model. Everything
else in the package is model-agnostic and takes a ``RobotModel``.
"""

from __future__ import annotations

from dataclasses import dataclass

from pysoarm.constants import DEFAULT_BAUDRATE
from pysoarm.exceptions import ConfigurationError

__all__ = ["MODELS", "RobotModel", "get_model"]


@dataclass(frozen=True, slots=True)
class RobotModel:
    """Static description of an arm variant.

    Args:
        name: Short identifier, e.g. ``"so101"``.
        joints: Joint names in **control order**; this order defines every array in the API.
        motor_ids: Servo id for each joint, same order as ``joints``.
        gripper_joint: Name of the gripper joint, if the arm has one.
        baudrate: Bus speed for this model.
        servo_model: Servo model name understood by the transport (``"sts3215"``).
        gripper_register_defaults: Registers written at connect time for the gripper only,
            as ``(register, value)`` pairs (torque/current caps that protect the cheap servo).

    Raises:
        ConfigurationError: If the description is inconsistent.
    """

    name: str
    joints: tuple[str, ...]
    motor_ids: tuple[int, ...]
    gripper_joint: str | None = None
    baudrate: int = DEFAULT_BAUDRATE
    servo_model: str = "sts3215"
    gripper_register_defaults: tuple[tuple[str, int], ...] = ()

    def __post_init__(self) -> None:
        if not self.joints:
            raise ConfigurationError(f"Model {self.name!r} has no joints")
        if len(self.joints) != len(self.motor_ids):
            raise ConfigurationError(
                f"Model {self.name!r}: {len(self.joints)} joints but {len(self.motor_ids)} ids"
            )
        if len(set(self.joints)) != len(self.joints):
            raise ConfigurationError(f"Model {self.name!r} has duplicate joint names")
        if len(set(self.motor_ids)) != len(self.motor_ids):
            raise ConfigurationError(f"Model {self.name!r} has duplicate motor ids")
        if self.gripper_joint is not None and self.gripper_joint not in self.joints:
            raise ConfigurationError(
                f"Model {self.name!r}: gripper_joint {self.gripper_joint!r} is not a joint"
            )

    @property
    def joint_count(self) -> int:
        """Number of joints on the arm."""
        return len(self.joints)

    @property
    def gripper_id(self) -> int | None:
        """Servo id of the gripper, or ``None`` when the arm has no gripper."""
        return None if self.gripper_joint is None else self.id_of(self.gripper_joint)

    def id_of(self, joint: str) -> int:
        """Servo id behind ``joint``.

        Raises:
            ConfigurationError: If the joint is not part of this model.
        """
        try:
            return self.motor_ids[self.joints.index(joint)]
        except ValueError as exc:
            raise ConfigurationError(
                f"Unknown joint {joint!r} for model {self.name!r}; known: {self.joints}"
            ) from exc

    def joint_of(self, motor_id: int) -> str:
        """Joint name behind ``motor_id``.

        Raises:
            ConfigurationError: If the id is not part of this model.
        """
        try:
            return self.joints[self.motor_ids.index(motor_id)]
        except ValueError as exc:
            raise ConfigurationError(
                f"Unknown motor id {motor_id} for model {self.name!r}; known: {self.motor_ids}"
            ) from exc

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view, handy for CLI output."""
        return {
            "name": self.name,
            "joints": list(self.joints),
            "motor_ids": list(self.motor_ids),
            "gripper_joint": self.gripper_joint,
            "baudrate": self.baudrate,
            "servo_model": self.servo_model,
        }


#: Populated by :mod:`pysoarm.models.so101`; kept here so both modules avoid a cycle.
MODELS: dict[str, RobotModel] = {}


def get_model(name: str) -> RobotModel:
    """Look up a model by name (case-insensitive).

    Raises:
        ConfigurationError: If the model is unknown.
    """
    try:
        return MODELS[name.lower()]
    except KeyError as exc:
        raise ConfigurationError(f"Unknown model {name!r}; known: {sorted(MODELS)}") from exc
