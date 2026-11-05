"""Tiny helpers shared by the examples (not part of the public package API)."""

from __future__ import annotations

import argparse
import os

from pysoarm import FakeBus, SOArm


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    """Add ``--port``, ``--backend``, ``--calibration`` and ``--fake``."""
    parser.add_argument(
        "--port",
        default=os.environ.get("PYSOARM_HARDWARE_PORT", "/dev/ttyACM0"),
        help="serial device (default: $PYSOARM_HARDWARE_PORT or /dev/ttyACM0)",
    )
    parser.add_argument("--backend", default="auto", choices=("auto", "scs", "rustypot"))
    parser.add_argument(
        "--calibration",
        default=os.environ.get("PYSOARM_CALIBRATION"),
        help="LeRobot JSON (default: $PYSOARM_CALIBRATION, else read from the motors)",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="run against an in-memory bus: no hardware, no serial port",
    )


def open_arm(args: argparse.Namespace, **kwargs: object) -> SOArm:
    """Build an arm for hardware or for the fake bus."""
    if args.fake:
        bus = FakeBus()
        for motor_id in bus.motor_ids:
            bus.set_register(motor_id, "min_position_limit", 800)
            bus.set_register(motor_id, "max_position_limit", 3200)
        arm = SOArm(bus=bus, **kwargs)  # type: ignore[arg-type]
    else:
        arm = SOArm(
            args.port,
            backend=args.backend,
            calibration=args.calibration,
            **kwargs,  # type: ignore[arg-type]
        )
    arm.connect()
    return arm
