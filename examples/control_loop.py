"""A 200 Hz control loop with a callback, pylibfranka-style.

Intended behaviour::

    arm.run_control_loop(lambda state, t: hold(state.q_deg), hz=200, duration=3.0)

and print the cycle-time statistics (mean / p99 / overruns) that the loop records. The
measured budget on reference hardware is ~1.2 ms per read+write step.
"""

from __future__ import annotations

raise NotImplementedError("example lands with the 0.1.0 implementation")
