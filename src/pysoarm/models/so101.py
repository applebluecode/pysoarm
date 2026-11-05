"""SO-101 / SO-100 description.

Six STS3215 servos, ids 1-6, in this order (the order every array in this package uses)::

    shoulder_pan  shoulder_lift  elbow_flex  wrist_flex  wrist_roll  gripper

SO-100 shares the bus layout; only the mechanics differ, so it is registered as an alias with
its own name. LeRobot's follower and leader use the same layout — the leader has no torque and
is read-only from this package's point of view.
"""

from __future__ import annotations

from pysoarm.constants import (
    GRIPPER_MAX_TORQUE_LIMIT,
    GRIPPER_OVERLOAD_TORQUE,
    GRIPPER_PROTECTION_CURRENT,
)
from pysoarm.models.base import MODELS, RobotModel

__all__ = ["SO100", "SO101", "SO_JOINTS"]

#: Joint order shared by SO-100 and SO-101.
SO_JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

#: Half torque / half current caps on the gripper, to avoid burning the cheap servo
#: (LeRobot's SO-101 defaults).
_GRIPPER_PROTECTION: tuple[tuple[str, int], ...] = (
    ("max_torque_limit", GRIPPER_MAX_TORQUE_LIMIT),
    ("protection_current", GRIPPER_PROTECTION_CURRENT),
    ("overload_torque", GRIPPER_OVERLOAD_TORQUE),
)

SO101 = RobotModel(
    name="so101",
    joints=SO_JOINTS,
    motor_ids=(1, 2, 3, 4, 5, 6),
    gripper_joint="gripper",
    servo_model_number=777,
    gripper_register_defaults=_GRIPPER_PROTECTION,
)

SO100 = RobotModel(
    name="so100",
    joints=SO_JOINTS,
    motor_ids=(1, 2, 3, 4, 5, 6),
    gripper_joint="gripper",
    servo_model_number=777,
    gripper_register_defaults=_GRIPPER_PROTECTION,
)

MODELS.update(
    {
        "so101": SO101,
        "so101_follower": SO101,
        "so101_leader": SO101,
        "so100": SO100,
        "so100_follower": SO100,
        "so100_leader": SO100,
        "so_follower": SO101,
        "so_leader": SO101,
    }
)
