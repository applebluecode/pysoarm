"""Enable torque and hold the current pose.

The safest possible first program: the goal is the measured position, so nothing moves.

Intended behaviour: connect, ``read_once()``, ``write_goal(state.q_deg)``, then hold until
Ctrl+C, then ``stop()`` (torque off) from the ``finally`` block.
"""

from __future__ import annotations

raise NotImplementedError("example lands with the 0.1.0 implementation")
