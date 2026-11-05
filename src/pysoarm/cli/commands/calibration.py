"""Motor-side calibration commands: ``calibrate`` and ``calibration show|export``."""

from __future__ import annotations

import argparse
import json
import sys

from pysoarm.calibration import calibrate, read_calibration
from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_OK, EXIT_USAGE
from pysoarm.models.base import get_model

__all__ = ["_cmd_calibrate", "_cmd_calibration"]


def _cmd_calibrate(args: argparse.Namespace) -> int:
    model = get_model(args.model)
    bus = factories.make_bus(args)
    with bus:
        calibration = calibrate(
            bus,
            model,
            role=args.role,
            middle=args.middle,
            full_turn=args.full_turn,
            input_fn=lambda prompt: input(prompt) if sys.stdin.isatty() else "",
        )
    print(f"calibrated {model.name} ({args.role}); {len(calibration.joints)} joints")
    for joint, cal in calibration.joints.items():
        print(
            f"  {joint:<14} id={cal.motor_id} range=[{cal.range_min}, {cal.range_max}] "
            f"homing_offset={cal.homing_offset}"
        )
    output = args.output or (f"{args.id}.json" if args.id else None)
    if output is not None:
        written = calibration.to_json(output)
        print(f"wrote {written}")
    return EXIT_OK


def _cmd_calibration(args: argparse.Namespace) -> int:
    model = get_model(args.model)
    bus = factories.make_bus(args)
    with bus:
        calibration = read_calibration(bus, model)
    if args.action == "export":
        if args.output is None:
            print_error("calibration export needs --output PATH")
            return EXIT_USAGE
        written = calibration.to_json(args.output)
        print(f"wrote {written}")
        return EXIT_OK
    if args.json:
        print(json.dumps(calibration.to_dict(), indent=2))
        return EXIT_OK
    print(f"{'joint':<14} {'id':>3} {'min':>5} {'max':>5} {'offset':>7} {'drive':>5}")
    for joint, cal in calibration.joints.items():
        print(
            f"{joint:<14} {cal.motor_id:>3} {cal.range_min:>5} {cal.range_max:>5} "
            f"{cal.homing_offset:>7} {cal.drive_mode:>5}"
        )
    return EXIT_OK
