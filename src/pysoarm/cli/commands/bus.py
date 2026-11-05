"""Bus and snapshot commands: ``scan``, ``read``, ``monitor`` and ``hold``.

These commands are the read-mostly entry points. They know about presentation and the
``--json`` switch; transport construction lives in :mod:`pysoarm.cli.factories`.
"""

from __future__ import annotations

import argparse
import json
import time

from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_ERROR, EXIT_OK, EXIT_USAGE
from pysoarm.models.base import get_model

__all__ = ["_cmd_hold", "_cmd_monitor", "_cmd_read", "_cmd_scan"]


def _cmd_scan(args: argparse.Namespace) -> int:
    model = get_model(args.model)
    bus = factories.make_bus(args)
    with bus:
        found = bus.scan(model.motor_ids)
    if not found:
        print_error(f"no servos answered on {args.port}")
        return EXIT_ERROR
    if args.json:
        print(json.dumps({"port": args.port, "motors": found}, indent=2))
    else:
        print(f"{'id':>3}  model")
        for motor_id, model_number in sorted(found.items()):
            print(f"{motor_id:>3}  {model_number}")
    return EXIT_OK


def _cmd_read(args: argparse.Namespace) -> int:
    with factories.make_arm(args) as arm:
        state = arm.read_once(health=True)
    if args.json:
        print(json.dumps({"joints": list(arm.model.joints), **state.as_dict()}, indent=2))
    else:
        cells = " ".join(
            f"{joint}={value:7.2f}"
            for joint, value in zip(arm.model.joints, state.q_deg, strict=True)
        )
        print(cells)
        print(
            f"temp_c={max(state.temperature_c)} "
            f"voltage_v={min(state.voltage_v):.1f} "
            f"read_t={state.t:.3f}"
        )
    return EXIT_OK


def _cmd_monitor(args: argparse.Namespace) -> int:
    if args.hz <= 0:
        print_error("--hz must be positive")
        return EXIT_USAGE
    with factories.make_arm(args) as arm:
        print(" ".join(f"{joint:>13}" for joint in arm.model.joints) + "   max_temp  min_volt")
        try:
            while True:
                state = arm.read_once()
                cells = " ".join(f"{value:13.2f}" for value in state.q_deg)
                print(
                    f"{cells}   {max(state.temperature_c):8.0f}  {min(state.voltage_v):8.1f}",
                    end="\r",
                    flush=True,
                )
                time.sleep(1.0 / args.hz)
        except KeyboardInterrupt:
            print()
    return EXIT_OK


def _cmd_hold(args: argparse.Namespace) -> int:
    if not args.yes:
        print_error("refusing to enable torque without --yes: make sure the workspace is clear")
        return EXIT_USAGE
    with factories.make_arm(args) as arm:
        print(f"holding {arm.model.joint_count} joints; press Ctrl+C to release torque")
        try:
            while True:
                arm.hold()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
    print("torque disabled, port released")
    return EXIT_OK
