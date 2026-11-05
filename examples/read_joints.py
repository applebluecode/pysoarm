"""Read one snapshot from the arm and print it.

Run without hardware::

    python examples/read_joints.py --fake
"""

from __future__ import annotations

import argparse
import time

from _common import add_connection_args, open_arm


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    args = parser.parse_args()
    arm = open_arm(args)
    try:
        started = time.perf_counter()
        state = arm.read_once(health=True)
        elapsed_ms = (time.perf_counter() - started) * 1e3
        print("q_deg    ", [round(value, 2) for value in state.q_deg])
        print("temp_c   ", list(state.temperature_c))
        print("voltage_v", [round(value, 1) for value in state.voltage_v])
        print(f"read took {elapsed_ms:.2f} ms")
    finally:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
