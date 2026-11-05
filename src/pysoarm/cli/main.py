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

``doctor`` exists because the two most common failures are environmental: another process
(a suspended ``lerobot-*``) holding the tty, and ``ModemManager`` probing ``/dev/ttyACM*``.
With ``--stress`` it also reports per-motor timeouts, status bits and voltage/temperature.
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import importlib.util
import json
import math
import os
import shutil
import stat
import statistics
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import cast

from pysoarm._version import __version__
from pysoarm.arm import SOArm
from pysoarm.bus import BACKENDS, make_bus
from pysoarm.bus.base import Bus
from pysoarm.calibration import calibrate, read_calibration
from pysoarm.exceptions import ConfigurationError, PysoarmError
from pysoarm.kinematics.base import Kinematics, Pose
from pysoarm.models.base import RobotModel, get_model

__all__ = ["EXIT_ERROR", "EXIT_OK", "EXIT_USAGE", "build_parser", "main"]

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_USAGE = 2
#: Temperature above which ``doctor --stress`` warns, in degrees Celsius.
_HOT_MOTOR_C = 60.0


def build_parser() -> argparse.ArgumentParser:  # noqa: PLR0915 - one declarative table
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
            default="auto",
            help="transport: auto (default: rustypot if installed, else scs), scs or rustypot",
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

    calibrate_cmd = sub.add_parser("calibrate", help="run the motor-side calibration flow")
    add_common(calibrate_cmd, needs_calibration=False)
    calibrate_cmd.add_argument("--role", choices=("follower", "leader"), default="follower")
    calibrate_cmd.add_argument("--id", default=None, help="default output name (id.json)")
    calibrate_cmd.add_argument("--output", default=None, help="also write this JSON file")
    calibrate_cmd.add_argument(
        "--full-turn",
        action="store_true",
        help="use the recorded range for wrist_roll instead of forcing 0..4095",
    )

    calibration = sub.add_parser("calibration", help="inspect or export the motor calibration")
    calibration.add_argument("action", choices=("show", "export"))
    add_common(calibration, needs_calibration=False)
    calibration.add_argument("--output", default=None, help="destination JSON for 'export'")
    calibration.add_argument("--json", action="store_true", help="machine-readable output")

    bench = sub.add_parser("bench", help="measure read/write latency and loop overruns")
    add_common(bench, needs_calibration=False)
    bench.add_argument("--seconds", type=float, default=2.0, help="loop duration (default: 2)")
    bench.add_argument("--hz", type=float, default=200.0, help="loop rate (default: 200)")
    bench.add_argument("--samples", type=int, default=200, help="read/write samples (default: 200)")
    bench.add_argument("--json", action="store_true", help="machine-readable output")

    doctor = sub.add_parser("doctor", help="diagnose the environment for a port")
    doctor.add_argument("--port", required=True, help="serial device to inspect")
    doctor.add_argument("--model", default="so101", help="arm model (default: so101)")
    doctor.add_argument("--baudrate", type=int, default=None, help="override the baudrate")
    doctor.add_argument(
        "--backend", choices=BACKENDS, default="auto", help="transport for --stress"
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
        default=5.0,
        help="warn below this supply voltage (default: 5.0 V)",
    )

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


def _make_kinematics(args: argparse.Namespace) -> Kinematics:
    """Build the Cartesian solver for ``pose``/``ik`` (monkeypatched in tests)."""
    # lazy: the placo adapter needs numpy and the optional 'kinematics' extra
    module = importlib.import_module("pysoarm.kinematics")
    factory = cast("Callable[..., Kinematics]", module.PlacoKinematics)
    return factory(args.urdf, target_frame=args.target_frame)


def _joint_list(text: str) -> tuple[float, ...]:
    """Parse ``"0, -30 60"`` into a tuple of floats."""
    cleaned = text.replace(",", " ").split()
    try:
        return tuple(float(value) for value in cleaned)
    except ValueError as exc:
        raise ConfigurationError(f"Cannot parse joint values from {text!r}: {exc}") from exc


def _make_readonly_arm(args: argparse.Namespace) -> SOArm:
    """Arm used by ``pose --port``: no register writes, torque stays off."""
    return SOArm(
        args.port,
        model=args.model,
        backend=args.backend,
        calibration=args.calibration,
        configure_motors=False,
        enable_torque=False,
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


def _percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile (``0.0`` for an empty list)."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def _latency_summary(samples: list[float]) -> dict[str, float]:
    """Mean/p50/p95/max in milliseconds from second samples."""
    scale = 1e3
    return {
        "mean_ms": statistics.fmean(samples) * scale if samples else 0.0,
        "p50_ms": _percentile(samples, 0.50) * scale,
        "p95_ms": _percentile(samples, 0.95) * scale,
        "max_ms": max(samples, default=0.0) * scale,
    }


def _cmd_calibrate(args: argparse.Namespace) -> int:
    model = get_model(args.model)
    bus = _make_bus(args)
    with bus:
        calibration = calibrate(
            bus,
            model,
            role=args.role,
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
    bus = _make_bus(args)
    with bus:
        calibration = read_calibration(bus, model)
    if args.action == "export":
        if args.output is None:
            _print_error("calibration export needs --output PATH")
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


def _cmd_bench(args: argparse.Namespace) -> int:
    if args.seconds <= 0 or args.hz <= 0 or args.samples <= 0:
        _print_error("--seconds, --hz and --samples must be positive")
        return EXIT_USAGE
    model = get_model(args.model)
    ids = model.motor_ids
    bus = _make_bus(args)
    reads: list[float] = []
    writes: list[float] = []
    with bus:
        positions = bus.sync_read(ids, "present_position")
        for _ in range(args.samples):
            started = time.perf_counter()
            positions = bus.sync_read(ids, "present_position")
            reads.append(time.perf_counter() - started)
            started = time.perf_counter()
            bus.sync_write("goal_position", {mid: positions[mid] for mid in ids})
            writes.append(time.perf_counter() - started)

        period = 1.0 / args.hz
        started = time.perf_counter()
        ticks = 0
        overruns = 0
        while True:
            elapsed = time.perf_counter() - started
            if elapsed >= args.seconds:
                break
            positions = bus.sync_read(ids, "present_position")
            bus.sync_write("goal_position", positions)
            deadline = started + (ticks + 1) * period
            slack = deadline - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
            ticks += 1
        loop_elapsed = time.perf_counter() - started

    report = {
        "port": args.port,
        "backend": bus.backend,
        "read": _latency_summary(reads),
        "write": _latency_summary(writes),
        "loop": {
            "hz": args.hz,
            "ticks": ticks,
            "overruns": overruns,
            "elapsed_s": loop_elapsed,
            "mean_hz": ticks / loop_elapsed if loop_elapsed > 0 else 0.0,
        },
    }
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        read, write, loop = report["read"], report["write"], report["loop"]
        print(f"backend {report['backend']}")
        print(
            f"read6   mean {read['mean_ms']:.2f} ms  p50 {read['p50_ms']:.2f}  "
            f"p95 {read['p95_ms']:.2f}  max {read['max_ms']:.2f}"
        )
        print(
            f"write6  mean {write['mean_ms']:.2f} ms  p50 {write['p50_ms']:.2f}  "
            f"p95 {write['p95_ms']:.2f}  max {write['max_ms']:.2f}"
        )
        print(
            f"loop    {loop['hz']:.0f} Hz target, {loop['mean_hz']:.1f} Hz actual, "
            f"{loop['ticks']} ticks, {loop['overruns']} overruns"
        )
    return EXIT_OK


def _stress(
    bus: Bus,
    model: RobotModel,
    *,
    seconds: float,
    min_voltage: float,
    clock: Callable[[], float] = time.monotonic,
) -> tuple[list[str], dict[int, int], dict[int, set[str]], dict[str, float]]:
    """Probe every motor individually for ``seconds`` and summarise the failures."""
    ids = model.motor_ids
    volts = bus.sync_read(ids, "present_voltage")
    temps = bus.sync_read(ids, "present_temperature")
    warnings: list[str] = []
    for mid in ids:
        voltage = volts[mid] * 0.1
        if voltage < min_voltage:
            warnings.append(f"motor {mid}: {voltage:.1f} V is below {min_voltage:.1f} V")
        if temps[mid] >= _HOT_MOTOR_C:
            warnings.append(f"motor {mid}: {temps[mid]} C is hot")
    timeouts: dict[int, int] = dict.fromkeys(ids, 0)
    flags: dict[int, set[str]] = {mid: set() for mid in ids}
    samples: list[float] = []
    deadline = clock() + seconds
    while clock() < deadline:
        for mid in ids:
            started = clock()
            try:
                bus.read(mid, "present_position", retries=0)
            except PysoarmError as exc:
                timeouts[mid] += 1
                flags[mid].update(getattr(exc, "flags", set()))
            samples.append(clock() - started)
    return warnings, timeouts, flags, _latency_summary(samples)


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
    rules = Path("/etc/udev/rules.d/99-pysoarm.rules")
    checks.append(
        (
            "udev rules installed",
            rules.is_file(),
            str(rules) if rules.is_file() else "run scripts/install_udev_rules.sh",
            False,
        )
    )
    rustypot = importlib.util.find_spec("rustypot") is not None
    checks.append(
        (
            "rustypot available",
            rustypot,
            "installed" if rustypot else 'pip install "pysoarm[rustypot]"',
            False,
        )
    )

    if args.stress > 0:
        model = get_model(args.model)
        try:
            bus = _make_bus(args)
            with bus:
                warnings, timeouts, flags, latency = _stress(
                    bus, model, seconds=args.stress, min_voltage=args.min_voltage
                )
        except PysoarmError as exc:
            checks.append(("bus probe", False, str(exc), False))
        else:
            wired = [mid for mid, count in timeouts.items() if count]
            checks.append(
                (
                    "no read timeouts",
                    not wired,
                    ", ".join(f"{mid}: {timeouts[mid]}" for mid in wired) or "clean",
                    False,
                )
            )
            checks.append(
                (
                    "no status alarms",
                    not any(flags.values()),
                    ", ".join(f"{mid}: {sorted(flags[mid])}" for mid in flags if flags[mid])
                    or "clean",
                    False,
                )
            )
            print(
                f"info backend {bus.backend}; read latency "
                f"mean {latency['mean_ms']:.2f} ms p95 {latency['p95_ms']:.2f} ms"
            )
            for warning in warnings:
                print(f"warn {warning}")

    for name, ok, detail, fatal in checks:
        status = "ok  " if ok else ("FAIL" if fatal else "warn")
        print(f"{status} {name:<32} {detail}")
    failed = [name for name, ok, _, fatal in checks if fatal and not ok]
    if failed:
        _print_error(f"{len(failed)} check(s) failed")
        return EXIT_ERROR
    return EXIT_OK


def _cmd_pose(args: argparse.Namespace) -> int:
    kinematics = _make_kinematics(args)
    if args.q is not None:
        q_deg = _joint_list(args.q)
    elif args.port is not None:
        with _make_readonly_arm(args) as arm:
            q_deg = arm.read_once(health=False).q_deg
    else:
        _print_error("pass either --q or --port")
        return EXIT_USAGE
    pose = kinematics.forward(q_deg)
    if args.json:
        print(json.dumps({"joints_deg": list(q_deg), **pose.as_dict()}, indent=2))
    else:
        x, y, z, roll, pitch, yaw = pose.to_xyz_rpy()
        print(f"xyz_m   {x:8.4f} {y:8.4f} {z:8.4f}")
        print(f"rpy_deg {roll:8.2f} {pitch:8.2f} {yaw:8.2f}")
    return EXIT_OK


def _cmd_ik(args: argparse.Namespace) -> int:
    kinematics = _make_kinematics(args)
    roll, pitch, yaw = args.rpy
    target = Pose.from_xyz_rpy(*args.xyz, roll=roll, pitch=pitch, yaw=yaw)
    seed = _joint_list(args.seed) if args.seed else None
    solution = kinematics.inverse(target, seed_deg=seed)
    if args.json:
        print(json.dumps({"joints_deg": list(solution)}, indent=2))
    else:
        print(" ".join(f"{value:8.2f}" for value in solution))
    return EXIT_OK


_HANDLERS = {
    "scan": _cmd_scan,
    "read": _cmd_read,
    "monitor": _cmd_monitor,
    "hold": _cmd_hold,
    "calibrate": _cmd_calibrate,
    "calibration": _cmd_calibration,
    "bench": _cmd_bench,
    "doctor": _cmd_doctor,
    "pose": _cmd_pose,
    "ik": _cmd_ik,
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
