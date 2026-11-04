"""Kinematics protocol and pose types.

.. todo::
   Define ``Pose`` (position in metres + quaternion, documented frame conventions) and the
   ``Kinematics`` protocol. Solvers must be pure and must report ``NotReachable`` instead of
   returning a silently saturated joint vector.
"""

from __future__ import annotations

__all__: list[str] = []
