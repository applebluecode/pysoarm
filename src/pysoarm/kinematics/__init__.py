"""Optional forward/inverse kinematics on top of a user-supplied URDF.

Kept behind an extra (``pip install "pysoarm[kinematics]"``) and out of the control loop: the
position loop works in joint space, kinematics only converts to and from Cartesian goals. A
SO-101 URDF is **not** shipped — licensing and the accuracy of community meshes vary, so pass
your own with ``--urdf`` / ``PlacoKinematics(urdf_path=...)``::

    from pysoarm import SOArm
    from pysoarm.kinematics import PlacoKinematics

    arm = SOArm("/dev/so101_follower", kinematics=PlacoKinematics("so101.urdf"))
    pose = arm.forward()            # current end-effector pose
    arm.move_to_pose(pose)          # IK + joint-space move, same safety guards

Importing this package needs no optional dependency: :class:`Pose` and the
:class:`Kinematics` protocol are pure Python, and the placo adapter is loaded lazily so that
``import pysoarm`` stays dependency-free.
"""

from __future__ import annotations

import importlib
from typing import TYPE_CHECKING, Any

from pysoarm.kinematics.base import Kinematics, Pose

if TYPE_CHECKING:  # for type checkers only; no import at runtime
    from pysoarm.kinematics.placo_solver import PlacoKinematics, require_placo

__all__ = ["Kinematics", "PlacoKinematics", "Pose", "require_placo"]

_LAZY = {"PlacoKinematics", "require_placo"}


def __getattr__(name: str) -> Any:
    """Import the placo adapter on first use (it needs numpy and the ``kinematics`` extra)."""
    if name in _LAZY:
        placo_solver = importlib.import_module("pysoarm.kinematics.placo_solver")
        return getattr(placo_solver, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
