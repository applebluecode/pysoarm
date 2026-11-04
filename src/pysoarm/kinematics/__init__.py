"""Optional forward/inverse kinematics.

Kept behind an extra (``pip install "pysoarm[kinematics]"``) and out of the control loop:
the position loop works in joint space, kinematics only converts to and from Cartesian
goals. A SO-101 URDF is *not* shipped (licensing and accuracy of the community meshes vary);
supply your own and pass its path.

.. todo::
   Add a ``Kinematics`` protocol (``forward(q_deg) -> Pose``,
   ``inverse(pose, seed) -> np.ndarray``) and a ``PlacoKinematics`` implementation.
"""

from __future__ import annotations

__all__: list[str] = []
