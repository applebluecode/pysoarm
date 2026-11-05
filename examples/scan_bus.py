"""Scan a serial port for STS3215 servos.

python examples/scan_bus.py --port /dev/ttyACM0
python examples/scan_bus.py --fake
"""

from __future__ import annotations

import argparse
import os

from pysoarm import FakeBus, make_bus

MODEL_NAMES = {777: "STS3215"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=os.environ.get("PYSOARM_HARDWARE_PORT", "/dev/ttyACM0"),
        help="serial device",
    )
    parser.add_argument("--backend", default="auto", choices=("auto", "scs", "rustypot"))
    parser.add_argument("--fake", action="store_true", help="scan an in-memory bus instead")
    args = parser.parse_args()
    bus = FakeBus() if args.fake else make_bus(args.port, range(1, 7), backend=args.backend)
    try:
        with bus:
            found = bus.scan()
    except OSError as exc:
        print(f"cannot open {args.port!r}: {exc}")
        print("hint: a suspended lerobot-* process may still hold the port")
        return 1
    if not found:
        print(f"no servos answered on {args.port}")
        return 1
    for motor_id, model_number in sorted(found.items()):
        print(f"id {motor_id}: model {model_number} ({MODEL_NAMES.get(model_number, '?')})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
