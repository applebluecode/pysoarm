"""Argument parser for the ``pysoarm`` command line.

This module changes when the CLI surface changes (commands, options, help text). It owns no
command behaviour: each handler lives in :mod:`pysoarm.cli.commands`. The parser is assembled
per command family so adding a command does not turn :func:`build_parser` into a long script.
"""

from __future__ import annotations

import argparse

from pysoarm._version import __version__
from pysoarm.bus import BACKENDS

__all__ = ["build_parser"]


def _add_common(target: argparse.ArgumentParser, *, needs_calibration: bool) -> None:
    """Add the port/model/backend options shared by most commands."""
    target.add_argument("--port", required=True, help="serial device, e.g. /dev/so101_follower")
    target.add_argument("--model", default="so101", help="arm model (default: so101)")
    target.add_argument("--baudrate", type=int, default=None, help="override the baudrate")
    target.add_argument(
        "--backend",
        choices=BACKENDS,
        default="auto",
        help="transport: auto (default: rustypot if installed, else scs), scs or rustypot",
    )
    if needs_calibration:
        target.add_argument(
            "--calibration",
            default=None,
            help="LeRobot-compatible calibration JSON; omit to read it from the motors",
        )


def _add_bus_commands(sub: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    scan = sub.add_parser("scan", help="list the servos answering on a port")
    _add_common(scan, needs_calibration=False)
    scan.add_argument("--json", action="store_true", help="machine-readable output")

    read = sub.add_parser("read", help="print one snapshot")
    _add_common(read, needs_calibration=True)
    read.add_argument("--json", action="store_true", help="machine-readable output")

    monitor = sub.add_parser("monitor", help="print a snapshot every 1/hz seconds")
    _add_common(monitor, needs_calibration=True)
    monitor.add_argument("--hz", type=float, default=2.0, help="refresh rate (default: 2)")

    hold = sub.add_parser("hold", help="enable torque and hold the measured pose")
    _add_common(hold, needs_calibration=True)
    hold.add_argument("--yes", action="store_true", help="confirm that torque will be enabled")


def _add_kinematics_commands(sub: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    pose = sub.add_parser("pose", help="end-effector pose for a joint configuration or the arm")
    pose.add_argument("--urdf", required=True, help="URDF of the arm (not shipped)")
    pose.add_argument("--target-frame", default="tool0", help="end-effector frame (default: tool0)")
    pose.add_argument("--q", default=None, help="joint values in degrees, e.g. '0,-30,60,0,90,0'")
    pose.add_argument("--port", default=None, help="read the joints from this port instead of --q")
    pose.add_argument("--model", default="so101", help="arm model (default: so101)")
    pose.add_argument("--backend", choices=BACKENDS, default="scs", help="transport for --port")
    pose.add_argument("--calibration", default=None, help="calibration JSON for --port")
    pose.add_argument("--json", action="store_true", help="machine-readable output")

    ik = sub.add_parser("ik", help="inverse kinematics, dry run (no hardware)")
    ik.add_argument("--urdf", required=True, help="URDF of the arm (not shipped)")
    ik.add_argument("--target-frame", default="tool0", help="end-effector frame (default: tool0)")
    ik.add_argument(
        "--xyz",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="target position in metres",
    )
    ik.add_argument(
        "--rpy",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("R", "P", "Y"),
        help="target orientation in degrees",
    )
    ik.add_argument("--seed", default=None, help="IK seed joint values in degrees")
    ik.add_argument("--json", action="store_true", help="machine-readable output")


def _add_calibration_commands(sub: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    calibrate_cmd = sub.add_parser("calibrate", help="run the motor-side calibration flow")
    _add_common(calibrate_cmd, needs_calibration=False)
    calibrate_cmd.add_argument("--role", choices=("follower", "leader"), default="follower")
    calibrate_cmd.add_argument("--id", default=None, help="default output name (id.json)")
    calibrate_cmd.add_argument("--output", default=None, help="also write this JSON file")
    calibrate_cmd.add_argument(
        "--no-middle",
        dest="middle",
        action="store_false",
        help=(
            "skip the LeRobot-style 'move to mid-travel' stage and home at the current "
            "pose (pre-0.1.3 behaviour)"
        ),
    )
    calibrate_cmd.add_argument(
        "--full-turn",
        action="store_true",
        help="use the recorded range for wrist_roll instead of forcing 0..4095",
    )

    calibration = sub.add_parser("calibration", help="inspect or export the motor calibration")
    calibration.add_argument("action", choices=("show", "export"))
    _add_common(calibration, needs_calibration=False)
    calibration.add_argument("--output", default=None, help="destination JSON for 'export'")
    calibration.add_argument("--json", action="store_true", help="machine-readable output")


def _add_bench_command(sub: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    bench = sub.add_parser("bench", help="measure read/write latency and loop overruns")
    _add_common(bench, needs_calibration=False)
    bench.add_argument("--seconds", type=float, default=2.0, help="loop duration (default: 2)")
    bench.add_argument("--hz", type=float, default=200.0, help="loop rate (default: 200)")
    bench.add_argument("--samples", type=int, default=200, help="read/write samples (default: 200)")
    bench.add_argument("--json", action="store_true", help="machine-readable output")


def _add_doctor_command(sub: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    doctor = sub.add_parser("doctor", help="diagnose the environment for a port")
    doctor.add_argument("--port", required=True, help="serial device to inspect")
    doctor.add_argument("--model", default="so101", help="arm model (default: so101)")
    doctor.add_argument("--baudrate", type=int, default=None, help="override the baudrate")
    doctor.add_argument(
        "--backend", choices=BACKENDS, default="auto", help="transport for --stress"
    )
    doctor.add_argument(
        "--role",
        choices=("follower", "leader"),
        default="follower",
        help="arm role; sets the default supply-voltage threshold (default: follower)",
    )
    doctor.add_argument(
        "--stress",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="also probe every motor for this many seconds (0 = off)",
    )
    doctor.add_argument(
        "--min-voltage",
        type=float,
        default=None,
        metavar="VOLTS",
        help=("warn below this supply voltage (default: 10.0 V follower, 4.0 V leader)"),
    )
    doctor.add_argument("--json", action="store_true", help="machine-readable output")


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser (kept separate so tests can inspect it)."""
    parser = argparse.ArgumentParser(
        prog="pysoarm",
        description="Lean, hardware-level control of SO-100/SO-101 arms (Feetech STS3215).",
        epilog=(
            "Exit codes: 0 success, 1 runtime error (bus/communication/safety), 2 usage error."
        ),
    )
    parser.add_argument("--version", action="version", version=f"pysoarm {__version__}")
    sub = parser.add_subparsers(dest="command", required=True, metavar="COMMAND")
    _add_bus_commands(sub)
    _add_kinematics_commands(sub)
    _add_calibration_commands(sub)
    _add_bench_command(sub)
    _add_doctor_command(sub)
    return parser
