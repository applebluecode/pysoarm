"""Smooth minimum-jerk move to a target joint configuration.

Holds for a second, moves to ``--target`` in ``--duration`` seconds, holds again, then
releases torque. On the fake bus this runs offline:

    python examples/move_to.py --fake
"""

from __future__ import annotations

import argparse
import statistics
import time

from _common import add_connection_args, open_arm

TARGET_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 40.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--target",
        default=",".join(str(value) for value in TARGET_DEG),
        help="comma-separated joint targets in degrees",
    )
    args = parser.parse_args()
    target = tuple(float(value) for value in args.target.split(","))
    arm = open_arm(args)
    try:
        time.sleep(0.2)
        state = arm.move_to(target, duration=args.duration)
        time.sleep(0.2)
        error = statistics.fmean(
            abs(wanted - actual) for wanted, actual in zip(target, state.q_deg, strict=True)
        )
        print("target   ", [round(value, 2) for value in target])
        print("measured ", [round(value, 2) for value in state.q_deg])
        print(f"mean endpoint error {error:.2f} deg")
    finally:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
