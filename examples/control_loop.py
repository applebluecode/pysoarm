"""A 200 Hz control loop with a callback, pylibfranka-style.

The callback returns the goal for the next tick; returning ``None`` holds without writing.
Statistics include overruns and the command latency distribution:

    python examples/control_loop.py --fake --seconds 1
"""

from __future__ import annotations

import argparse

from _common import add_connection_args, open_arm
from pysoarm import ArmState


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument("--hz", type=float, default=200.0)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    arm = open_arm(args)

    def hold(state: ArmState, elapsed: float) -> tuple[float, ...]:
        del elapsed
        return state.q_deg

    try:
        stats = arm.run_control_loop(hold, hz=args.hz, duration=args.seconds)
        print(stats.as_dict())
    finally:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
