"""``pysoarm`` CLI entry point.

Sub-commands::

    pysoarm scan    --port /dev/so101_follower      # ids + model numbers on the bus
    pysoarm read    --port ... --json               # one snapshot, machine readable
    pysoarm monitor --port ... --hz 5               # live positions/temps/voltages
    pysoarm hold    --port ... --yes                # enable torque, hold the current pose
    pysoarm calibrate --port ... --role follower    # motor-side calibration flow
    pysoarm calibration show --port ...             # print the calibration stored in the motors
    pysoarm calibration export --port ... --output f.json
    pysoarm bench   --port ... --seconds 5          # read/write latency and loop overruns
    pysoarm doctor  --port ... [--stress 3]         # environment + electrical self-check

The parser lives in :mod:`pysoarm.cli.parser`; command implementations live in
:mod:`pysoarm.cli.commands`. This module only wires handlers and maps domain errors to stable
exit codes, so adding a command does not grow a central implementation file.
"""

from __future__ import annotations

from collections.abc import Sequence

from pysoarm.cli.commands import bench, bus, calibration, diagnostics, kinematics, print_error
from pysoarm.cli.constants import EXIT_ERROR, EXIT_OK, EXIT_USAGE
from pysoarm.cli.parser import build_parser
from pysoarm.exceptions import PysoarmError

__all__ = ["EXIT_ERROR", "EXIT_OK", "EXIT_USAGE", "build_parser", "main"]

_HANDLERS = {
    "scan": bus._cmd_scan,
    "read": bus._cmd_read,
    "monitor": bus._cmd_monitor,
    "hold": bus._cmd_hold,
    "calibrate": calibration._cmd_calibrate,
    "calibration": calibration._cmd_calibration,
    "bench": bench._cmd_bench,
    "doctor": diagnostics._cmd_doctor,
    "pose": kinematics._cmd_pose,
    "ik": kinematics._cmd_ik,
}


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI.

    Args:
        argv: Argument list; defaults to ``sys.argv[1:]``.

    Returns:
        Process exit code: 0 success, 1 runtime error, 2 usage error.
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    handler = _HANDLERS[args.command]
    try:
        return handler(args)
    except PysoarmError as exc:
        print_error(str(exc))
        return EXIT_ERROR
    except (OSError, ValueError) as exc:
        print_error(f"{type(exc).__name__}: {exc}")
        return EXIT_ERROR
