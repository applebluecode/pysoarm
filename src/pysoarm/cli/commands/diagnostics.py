"""Environment and electrical diagnostics: ``doctor`` and its stress probe."""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import json
import os
import shutil
import stat
import subprocess
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from pysoarm.bus.base import Bus
from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_ERROR, EXIT_OK, HOT_MOTOR_C
from pysoarm.cli.metrics import latency_summary
from pysoarm.constants import VOLTAGE_MIN_FOLLOWER_V, VOLTAGE_MIN_LEADER_V
from pysoarm.exceptions import PysoarmError
from pysoarm.models.base import RobotModel, get_model

__all__ = ["StressResult", "_cmd_doctor", "_port_holders", "_service_active", "_stress"]


@dataclass(frozen=True, slots=True)
class StressResult:
    """Outcome of :func:`_stress`, with failures split by cause.

    Args:
        warnings: Electrical warnings gathered before the probe (voltage/temperature).
        timeouts: Per motor, reads that did not answer at all (``status is None``).
        status_alarms: Per motor, counts of the named alarm bits returned in a status packet.
        latency: Summary of successful reads (milliseconds).
        failed_latency: Summary of failed reads, so a fast timeout is distinguishable from a
            slow one.
    """

    warnings: tuple[str, ...]
    timeouts: dict[int, int]
    status_alarms: dict[int, dict[str, int]]
    latency: dict[str, float]
    failed_latency: dict[str, float]

    @property
    def alarm_counts(self) -> dict[str, int]:
        """Alarm bits summed over motors, e.g. ``{"voltage": 2}``."""
        counts: dict[str, int] = {}
        for alarms in self.status_alarms.values():
            for flag, count in alarms.items():
                counts[flag] = counts.get(flag, 0) + count
        return counts

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view with stable field names."""
        ordered_timeouts = {str(mid): count for mid, count in sorted(self.timeouts.items())}
        ordered_alarms = {
            str(mid): dict(sorted(alarms.items()))
            for mid, alarms in sorted(self.status_alarms.items())
            if alarms
        }
        return {
            "warnings": list(self.warnings),
            "timeouts": ordered_timeouts,
            "status_alarms": ordered_alarms,
            "alarm_counts": dict(sorted(self.alarm_counts.items())),
            "latency": self.latency,
            "failed_latency": self.failed_latency,
        }


def _stress(
    bus: Bus,
    model: RobotModel,
    *,
    seconds: float,
    min_voltage: float,
    clock: Callable[[], float] = time.monotonic,
) -> StressResult:
    """Probe every motor individually for ``seconds`` and summarise the failures.

    A read can fail for two different reasons and the report keeps them apart: a **timeout**
    is no answer at all (``CommunicationError.status is None``), a **status alarm** is an
    answer whose ``ERROR`` byte has an alarm bit set. The pre-0.1.2 probe counted both as
    "timeouts", which double-reported supply sags as lost packets.
    """
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
    status_alarms: dict[int, dict[str, int]] = {mid: {} for mid in ids}
    samples: list[float] = []
    failed_samples: list[float] = []
    deadline = clock() + seconds
    while clock() < deadline:
        for mid in ids:
            started = clock()
            try:
                bus.read(mid, "present_position", retries=0)
            except PysoarmError as exc:
                failed_samples.append(clock() - started)
                status = getattr(exc, "status", None)
                if status is None:
                    timeouts[mid] += 1
                else:
                    for flag in getattr(exc, "flags", frozenset()):
                        alarms = status_alarms[mid]
                        alarms[flag] = alarms.get(flag, 0) + 1
            else:
                samples.append(clock() - started)
    return StressResult(
        warnings=tuple(warnings),
        timeouts=timeouts,
        status_alarms=status_alarms,
        latency=latency_summary(samples),
        failed_latency=latency_summary(failed_samples),
    )


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
    role = getattr(args, "role", "follower")
    min_voltage = (
        args.min_voltage
        if getattr(args, "min_voltage", None) is not None
        else (VOLTAGE_MIN_FOLLOWER_V if role == "follower" else VOLTAGE_MIN_LEADER_V)
    )
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

    stress: StressResult | None = None
    backend: str | None = None
    if args.stress > 0:
        model = get_model(args.model)
        try:
            bus = factories.make_bus(args)
            with bus:
                backend = bus.backend
                stress = _stress(bus, model, seconds=args.stress, min_voltage=min_voltage)
        except PysoarmError as exc:
            checks.append(("bus probe", False, str(exc), False))
        else:
            lost = [mid for mid, count in stress.timeouts.items() if count]
            checks.append(
                (
                    "no read timeouts",
                    not lost,
                    ", ".join(f"{mid}: {stress.timeouts[mid]}" for mid in lost) or "clean",
                    False,
                )
            )
            alarmed = [mid for mid, alarms in stress.status_alarms.items() if alarms]
            checks.append(
                (
                    "no status alarms",
                    not alarmed,
                    ", ".join(
                        f"{mid}: {dict(sorted(stress.status_alarms[mid].items()))}"
                        for mid in alarmed
                    )
                    or "clean",
                    False,
                )
            )
            if not args.json:
                print(
                    f"info backend {bus.backend}; read latency "
                    f"mean {stress.latency['mean_ms']:.2f} ms "
                    f"p95 {stress.latency['p95_ms']:.2f} ms"
                )
                for warning in stress.warnings:
                    print(f"warn {warning}")

    if args.json:
        payload = {
            "port": str(path),
            "role": role,
            "min_voltage_v": min_voltage,
            "backend": backend,
            "checks": [
                {
                    "name": name,
                    "ok": ok,
                    "detail": detail,
                    "fatal": fatal,
                    "status": "ok" if ok else ("fail" if fatal else "warn"),
                }
                for name, ok, detail, fatal in checks
            ],
        }
        payload.update(
            stress.as_dict()
            if stress is not None
            else {
                "warnings": [],
                "timeouts": {},
                "status_alarms": {},
                "alarm_counts": {},
                "latency": {},
                "failed_latency": {},
            }
        )
        print(json.dumps(payload, indent=2))
    else:
        for name, ok, detail, fatal in checks:
            status = "ok  " if ok else ("FAIL" if fatal else "warn")
            print(f"{status} {name:<32} {detail}")
    failed = [name for name, ok, _, fatal in checks if fatal and not ok]
    if failed:
        print_error(f"{len(failed)} check(s) failed")
        return EXIT_ERROR
    return EXIT_OK
