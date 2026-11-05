"""Environment and electrical diagnostics: ``doctor`` and its stress probe."""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import os
import shutil
import stat
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

from pysoarm.bus.base import Bus
from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_ERROR, EXIT_OK, HOT_MOTOR_C
from pysoarm.cli.metrics import latency_summary
from pysoarm.exceptions import PysoarmError
from pysoarm.models.base import RobotModel, get_model

__all__ = ["_cmd_doctor", "_port_holders", "_service_active", "_stress"]


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
        if temps[mid] >= HOT_MOTOR_C:
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
    return warnings, timeouts, flags, latency_summary(samples)


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
            bus = factories.make_bus(args)
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
        print_error(f"{len(failed)} check(s) failed")
        return EXIT_ERROR
    return EXIT_OK
