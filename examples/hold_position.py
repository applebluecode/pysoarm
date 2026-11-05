"""Enable torque and hold the current pose.

The safest first program: the goal is the measured position, so nothing moves. Press Ctrl+C
to release torque (``arm.stop()`` runs from the ``finally`` block).

    python examples/hold_position.py --fake
    python examples/hold_position.py --port /dev/ttyACM0 --calibration follower.json
"""

from __future__ import annotations

import argparse
import time

from _common import add_connection_args, open_arm


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument("--seconds", type=float, default=None, help="stop after N seconds")
    args = parser.parse_args()
    arm = open_arm(args)
    try:
        state = arm.read_once()
        print("holding", [round(value, 2) for value in state.q_deg])
        deadline = None if args.seconds is None else time.monotonic() + args.seconds
        while deadline is None or time.monotonic() < deadline:
            arm.hold()
    except KeyboardInterrupt:
        print("\nreleasing torque")
    finally:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
