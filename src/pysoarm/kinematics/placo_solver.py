"""Inverse/forward kinematics through ``placo`` (optional extra).

``placo`` builds a kinematic model from a URDF and solves frame tasks with a damped
least-squares step. The adapters here:

* work in **degrees and metres** at the boundary (radians internally, as placo expects);
* use a **small task gain** (``1e-3``) and iterate until the position residual is below
  ``tolerance_m`` — a gain of ``1.0`` overshoots and lands in a wrong branch on the reference
  two-link model, so the default is deliberately conservative;
* seed the solver from the current joint configuration (a Cartesian move from the current
  pose converges in a handful of iterations);
* raise :class:`~pysoarm.exceptions.NotReachableError` with the residual instead of returning
  a saturated joint vector.

Orientation is **not** constrained by default (``orientation_gain=0``): a 4-DoF arm cannot meet
a full 6-D pose, and silently trading position against orientation is worse than ignoring it.
Set ``orientation_gain`` on arms with enough degrees of freedom.
"""

from __future__ import annotations

import importlib
from collections.abc import Sequence
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import numpy.typing as npt

from pysoarm.exceptions import ConfigurationError, KinematicsError, NotReachableError
from pysoarm.kinematics.base import Pose

__all__ = ["PlacoKinematics", "require_placo"]

_INSTALL_HINT = 'pip install "pysoarm[kinematics]"'
#: Task gain used for the placo frame task; larger values overshoot (measured on 2-link).
DEFAULT_POSITION_GAIN = 1e-3
DEFAULT_MAX_ITERS = 200
DEFAULT_TOLERANCE_M = 1e-4


def _import_placo() -> Any:
    """Import the optional dependency (a seam, so tests can simulate it being absent)."""
    return importlib.import_module("placo")


def require_placo(module: ModuleType | Any | None = None) -> Any:
    """Return the ``placo`` module.

    Args:
        module: Injection seam for tests; returned unchanged when given.

    Raises:
        KinematicsError: If ``placo`` is not installed, with the install command.
    """
    if module is not None:
        return module
    try:
        return _import_placo()
    except ModuleNotFoundError as exc:
        raise KinematicsError(
            "PlacoKinematics needs the optional 'placo' dependency; "
            f"install it with {_INSTALL_HINT}."
        ) from exc


def _pose_to_matrix(pose: Pose) -> npt.NDArray[np.float64]:
    """4x4 homogeneous transform for ``pose`` (rotation from the quaternion)."""
    w, x, y, z = pose.qw, pose.qx, pose.qy, pose.qz
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = pose.position
    return matrix


def _matrix_to_pose(matrix: npt.NDArray[np.float64]) -> Pose:
    """Convert a 4x4 transform to a :class:`Pose` (quaternion from the rotation block)."""
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / scale
        qx = (rotation[2, 1] - rotation[1, 2]) * scale
        qy = (rotation[0, 2] - rotation[2, 0]) * scale
        qz = (rotation[1, 0] - rotation[0, 1]) * scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    return Pose(
        x=float(matrix[0, 3]),
        y=float(matrix[1, 3]),
        z=float(matrix[2, 3]),
        qw=float(qw),
        qx=float(qx),
        qy=float(qy),
        qz=float(qz),
    )


class PlacoKinematics:
    """Forward/inverse kinematics for a URDF, solved with ``placo``.

    Args:
        urdf_path: Path to the arm's URDF. Not shipped with pysoarm.
        target_frame: End-effector frame in the URDF (e.g. ``"tool0"`` or
            ``"gripper_frame_link"``).
        joint_names: Actuated joints, in the order of the arrays this object takes and
            returns. Defaults to every joint placo reports for the URDF.
        position_gain: Placo task gain for position; small values are stable.
        orientation_gain: Placo task gain for orientation, ``0`` to ignore orientation.
        max_iters: Solver iterations per :meth:`inverse` call.
        tolerance_m: Position residual at which :meth:`inverse` stops.

    Raises:
        KinematicsError: If ``placo`` is missing, the URDF cannot be loaded, or the target
            frame/joints are unknown.
        ConfigurationError: If a gain or tolerance is not positive.
    """

    def __init__(
        self,
        urdf_path: str | Path,
        *,
        target_frame: str = "tool0",
        joint_names: Sequence[str] | None = None,
        position_gain: float = DEFAULT_POSITION_GAIN,
        orientation_gain: float = 0.0,
        max_iters: int = DEFAULT_MAX_ITERS,
        tolerance_m: float = DEFAULT_TOLERANCE_M,
        placo_module: Any | None = None,
    ) -> None:
        if position_gain <= 0:
            raise ConfigurationError("position_gain must be positive")
        if orientation_gain < 0:
            raise ConfigurationError("orientation_gain must be non-negative")
        if max_iters < 1:
            raise ConfigurationError("max_iters must be at least 1")
        if tolerance_m <= 0:
            raise ConfigurationError("tolerance_m must be positive")
        self._placo = require_placo(placo_module)
        path = Path(urdf_path).expanduser()
        if not path.is_file():
            raise KinematicsError(f"URDF not found: {path}")
        try:
            self._robot = self._placo.RobotWrapper(str(path))
            self._solver = self._placo.KinematicsSolver(self._robot)
            self._solver.mask_fbase(True)  # the arm is bolted to the table
        except Exception as exc:  # placo raises RuntimeError/PyPlacoError on bad URDFs
            raise KinematicsError(f"Cannot load URDF {path}: {exc}") from exc
        self._target_frame = target_frame
        available = tuple(str(name) for name in self._robot.joint_names())
        if joint_names is None:
            self._joint_names = available
        else:
            self._joint_names = tuple(joint_names)
            unknown = [name for name in self._joint_names if name not in available]
            if unknown:
                raise KinematicsError(
                    f"Joints {unknown} are not in {path.name}; available: {available}"
                )
        try:
            self._task = self._solver.add_frame_task(target_frame, np.eye(4))
        except Exception as exc:
            raise KinematicsError(f"Frame {target_frame!r} is not in {path.name}: {exc}") from exc
        self._position_gain = position_gain
        self._orientation_gain = orientation_gain
        self._max_iters = max_iters
        self._tolerance_m = tolerance_m

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Actuated joint names, in the order used by these arrays."""
        return self._joint_names

    @property
    def target_frame(self) -> str:
        """End-effector frame name."""
        return self._target_frame

    def _set_joints(self, q_deg: Sequence[float]) -> None:
        if len(q_deg) != len(self._joint_names):
            raise ConfigurationError(
                f"Expected {len(self._joint_names)} joint values, got {len(q_deg)}"
            )
        for name, value in zip(self._joint_names, q_deg, strict=True):
            self._robot.set_joint(name, float(np.radians(value)))
        self._robot.update_kinematics()

    def _current_deg(self) -> tuple[float, ...]:
        return tuple(float(np.degrees(self._robot.get_joint(name))) for name in self._joint_names)

    def forward(self, q_deg: Sequence[float]) -> Pose:
        """End-effector pose for a joint configuration (degrees)."""
        self._set_joints(q_deg)
        return _matrix_to_pose(np.asarray(self._robot.get_T_world_frame(self._target_frame)))

    def inverse(
        self,
        pose: Pose,
        *,
        seed_deg: Sequence[float] | None = None,
        max_iters: int | None = None,
        tolerance_m: float | None = None,
    ) -> tuple[float, ...]:
        """Joint configuration (degrees) reaching ``pose``.

        Args:
            pose: Target end-effector pose.
            seed_deg: Initial guess; defaults to the solver's current joint values.
            max_iters: Override the configured iteration budget.
            tolerance_m: Override the configured position tolerance.

        Returns:
            Joint values in degrees, in :attr:`joint_names` order.

        Raises:
            NotReachableError: If the position residual stays above the tolerance.
        """
        target = np.asarray(pose.position, dtype=float)
        iterations = self._max_iters if max_iters is None else max_iters
        tolerance = self._tolerance_m if tolerance_m is None else tolerance_m
        if seed_deg is not None:
            self._set_joints(seed_deg)
        else:
            self._robot.update_kinematics()
        self._task.T_world_frame = _pose_to_matrix(pose)
        self._task.configure(
            self._target_frame, "soft", self._position_gain, self._orientation_gain
        )
        residual = float("inf")
        for _ in range(iterations):
            self._solver.solve(True)
            self._robot.update_kinematics()
            residual = float(
                np.linalg.norm(
                    np.asarray(self._robot.get_T_world_frame(self._target_frame))[:3, 3] - target
                )
            )
            if residual <= tolerance:
                return self._current_deg()
        raise NotReachableError(
            f"IK did not reach {tuple(round(v, 4) for v in pose.position)} m within "
            f"{iterations} iterations (residual {residual:.4f} m, tolerance {tolerance:.4f} m); "
            "the target may be outside the workspace or the seed may be in the wrong branch"
        )
