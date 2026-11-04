"""``pysoarm`` CLI entry point.

Sub-commands::

    pysoarm scan    --port /dev/so101_follower      # ids + model numbers on the bus
    pysoarm read    --port ... --json               # one snapshot, machine readable
    pysoarm monitor --port ... --hz 5               # live positions/temps/voltages
    pysoarm hold    --port ... --yes                # enable torque, hold the current pose
    pysoarm doctor  --port ...                      # port, permissions, other holders, ModemManager

``doctor`` exists because the two most common failures are environmental: another process
(a suspended ``lerobot-*``) holding the tty, and ``ModemManager`` probing ``/dev/ttyACM*``.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import shutil
import stat
import subprocess
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from pysoarm._version import __version__
from pysoarm.arm import SOArm
from pysoarm.bus import BACKENDS, make_bus
from pysoarm.bus.base import Bus
from pysoarm.exceptions import PysoarmError
from pysoarm.models.base import get_model

__all__ = ["EXIT_ERROR", "EXIT_OK", "EXIT_USAGE", "build_parser", "main"]

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_USAGE = 2


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

    def add_common(target: argparse.ArgumentParser, *, needs_calibration: bool) -> None:
        target.add_argument("--port", required=True, help="serial device, e.g. /dev/so101_follower")
        target.add_argument("--model", default="so101", help="arm model (default: so101)")
        target.add_argument("--baudrate", type=int, default=None, help="override the baudrate")
        target.add_argument(
            "--backend",
            choices=BACKENDS,
            default="scs",
            help="transport: scs (default), rustypot (needs the extra) or auto",
        )
        if needs_calibration:
            target.add_argument(
                "--calibration",
                default=None,
                help="LeRobot-compatible calibration JSON; omit to read it from the motors",
            )

    scan = sub.add_parser("scan", help="list the servos answering on a port")
    add_common(scan, needs_calibration=False)
    scan.add_argument("--json", action="store_true", help="machine-readable output")

    read = sub.add_parser("read", help="print one snapshot")
    add_common(read, needs_calibration=True)
    read.add_argument("--json", action="store_true", help="machine-readable output")

    monitor = sub.add_parser("monitor", help="print a snapshot every 1/hz seconds")
    add_common(monitor, needs_calibration=True)
    monitor.add_argument("--hz", type=float, default=2.0, help="refresh rate (default: 2)")

    hold = sub.add_parser("hold", help="enable torque and hold the measured pose")
    add_common(hold, needs_calibration=True)
    hold.add_argument("--yes", action="store_true", help="confirm that torque will be enabled")

    doctor = sub.add_parser("doctor", help="diagnose the environment for a port")
    doctor.add_argument("--port", required=True, help="serial device to inspect")

    return parser


def _make_bus(args: argparse.Namespace) -> Bus:
    """Build the transport for ``scan`` (monkeypatched in tests)."""
    model = get_model(args.model)
    return make_bus(
        args.port,
        model.motor_ids,
        backend=args.backend,
        servo_model=model.servo_model,
        baudrate=args.baudrate or model.baudrate,
    )


def _make_arm(args: argparse.Namespace) -> SOArm:
    """Build the arm for ``read``/``monitor``/``hold`` (monkeypatched in tests)."""
    return SOArm(
        args.port,
        model=args.model,
        backend=args.backend,
        calibration=args.calibration,
        baudrate=args.baudrate,
    )


def _print_error(message: str) -> None:
    print(f"pysoarm: {message}", file=sys.stderr)


def _cmd_scan(args: argparse.Namespace) -> int:
    model = get_model(args.model)
    bus = _make_bus(args)
    with bus:
        found = bus.scan(model.motor_ids)
    if not found:
        _print_error(f"no servos answered on {args.port}")
        return EXIT_ERROR
    if args.json:
        print(json.dumps({"port": args.port, "motors": found}, indent=2))
    else:
        print(f"{'id':>3}  model")
        for motor_id, model_number in sorted(found.items()):
            print(f"{motor_id:>3}  {model_number}")
    return EXIT_OK


def _cmd_read(args: argparse.Namespace) -> int:
    with _make_arm(args) as arm:
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
        _print_error("--hz must be positive")
        return EXIT_USAGE
    with _make_arm(args) as arm:
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
        _print_error("refusing to enable torque without --yes: make sure the workspace is clear")
        return EXIT_USAGE
    with _make_arm(args) as arm:
        print(f"holding {arm.model.joint_count} joints; press Ctrl+C to release torque")
        try:
            while True:
                arm.hold()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
    print("torque disabled, port released")
    return EXIT_OK


def _port_holders(path: Path) -> list[str]:
    """Processes holding ``path`` open (best effort, Linux only)."""
    holders: list[str] = []
    target = str(path.resolve())
    try:
        proc_entries = list(Path("/proc").iterdir())
    except OSError:
        return holders
    for entry in proc_entries:
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        fd_dir = entry / "fd"
        try:
            fds = list(fd_dir.iterdir())
        except OSError:
            continue
        for fd in fds:
            with contextlib.suppress(OSError):
                if fd.readlink() == Path(target):
                    with contextlib.suppress(OSError):
                        comm = entry.joinpath("comm").read_text().strip()
                        holders.append(f"{entry.name} ({comm})")
                    break
    return holders


def _service_active(name: str) -> bool | None:
    """Whether a systemd unit is active; ``None`` when systemd is unavailable."""
    if shutil.which("systemctl") is None:
        return None
    with contextlib.suppress(OSError, subprocess.SubprocessError):
        result = subprocess.run(
            ["systemctl", "is-active", name],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        return result.stdout.strip() == "active"
    return None


def _cmd_doctor(args: argparse.Namespace) -> int:
    path = Path(args.port)
    # (name, ok, detail, fatal): non-fatal findings are warnings and keep the exit code at 0.
    checks: list[tuple[str, bool, str, bool]] = []
    exists = path.exists()
    checks.append(("device exists", exists, str(path), True))
    if exists:
        mode = path.stat().st_mode
        checks.append(("is a character device", stat.S_ISCHR(mode), oct(mode), True))
        checks.append(
            ("readable and writable", os.access(path, os.R_OK | os.W_OK), "os.access", True)
        )
        holders = _port_holders(path)
        checks.append(
            ("not held by another process", not holders, ", ".join(holders) or "free", True)
        )
    modem_active = _service_active("ModemManager")
    if modem_active is not None:
        checks.append(
            (
                "ModemManager disabled",
                not modem_active,
                "active (may grab the tty on replug)" if modem_active else "inactive",
                False,
            )
        )

    for name, ok, detail, fatal in checks:
        status = "ok  " if ok else ("FAIL" if fatal else "warn")
        print(f"{status} {name:<32} {detail}")
    failed = [name for name, ok, _, fatal in checks if fatal and not ok]
    if failed:
        _print_error(f"{len(failed)} check(s) failed")
        return EXIT_ERROR
    return EXIT_OK


_HANDLERS = {
    "scan": _cmd_scan,
    "read": _cmd_read,
    "monitor": _cmd_monitor,
    "hold": _cmd_hold,
    "doctor": _cmd_doctor,
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
        _print_error(str(exc))
        return EXIT_ERROR
    except (OSError, ValueError) as exc:
        _print_error(f"{type(exc).__name__}: {exc}")
        return EXIT_ERROR
