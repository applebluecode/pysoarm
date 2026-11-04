"""Replay a recorded trajectory stored as ``npz(t, q_deg)``.

Intended behaviour: load the file, check it is monotonic in time and inside the calibrated
joint limits, then time-parameterise it at the loop rate using ``ResampledTrajectory``.
"""

from __future__ import annotations

raise NotImplementedError("example lands with the 0.1.0 implementation")
