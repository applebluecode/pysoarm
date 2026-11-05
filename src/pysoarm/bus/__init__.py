"""Bus transports: one protocol, several implementations.

The rest of the package only ever sees :class:`~pysoarm.bus.base.Bus`, so the transport can be
swapped without touching control logic — and every test above the transport layer runs
against :class:`~pysoarm.bus.fake.FakeBus`.

Backends
--------
``scs``
    Reference implementation: the SCS protocol on ``pyserial`` (the only hard dependency,
    ~2.2 ms to read six joints).
``rustypot``
    Optional Rust driver (the one LeRobot uses, ~1.0 ms for the same read). Requires the
    ``rustypot`` extra; ``make_bus(..., backend="rustypot")`` or ``--backend rustypot``.
``auto``
    ``rustypot`` when importable, ``scs`` otherwise.
``fake``
    Deterministic in-memory bus for tests (:class:`~pysoarm.bus.fake.FakeBus`).
"""

from __future__ import annotations

import importlib.util
from collections.abc import Sequence
from typing import Literal

from pysoarm.bus.base import Bus
from pysoarm.bus.fake import FakeBus
from pysoarm.bus.registers import REGISTERS, Register, RegisterKind, get_register
from pysoarm.bus.rustypot import RustypotBus
from pysoarm.bus.scs import ScsBus
from pysoarm.constants import DEFAULT_BAUDRATE, DEFAULT_RETRIES, DEFAULT_TIMEOUT_S
from pysoarm.exceptions import ConfigurationError

__all__ = [
    "REGISTERS",
    "Backend",
    "Bus",
    "FakeBus",
    "Register",
    "RegisterKind",
    "RustypotBus",
    "ScsBus",
    "get_register",
    "make_bus",
    "resolve_backend",
]

#: Transport selector accepted by :func:`make_bus`, :class:`~pysoarm.arm.SOArm` and the CLI.
Backend = Literal["scs", "rustypot", "auto"]

BACKENDS: tuple[str, ...] = ("scs", "rustypot", "auto")


def _rustypot_available() -> bool:
    try:
        return importlib.util.find_spec("rustypot") is not None
    except (ImportError, ValueError):  # a broken parent package counts as unavailable
        return False


def resolve_backend(backend: Backend) -> Literal["scs", "rustypot"]:
    """Resolve ``"auto"`` to a concrete backend.

    Args:
        backend: ``"scs"``, ``"rustypot"`` or ``"auto"``.

    Returns:
        ``"rustypot"`` when available and requested (or implied by ``"auto"``), else
        ``"scs"``.

    Raises:
        ConfigurationError: If ``backend`` is not one of the supported names.
    """
    if backend == "auto":
        return "rustypot" if _rustypot_available() else "scs"
    if backend in ("scs", "rustypot"):
        return backend
    raise ConfigurationError(f"Unknown backend {backend!r}; expected one of {BACKENDS}")


def make_bus(
    port: str,
    motor_ids: Sequence[int],
    *,
    backend: Backend = "auto",
    servo_model: str = "sts3215",
    baudrate: int = DEFAULT_BAUDRATE,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    retries: int = DEFAULT_RETRIES,
) -> Bus:
    """Build a transport for ``port``.

    Args:
        port: Serial device path.
        motor_ids: Servo ids on the bus.
        backend: ``"auto"`` (default: rustypot when importable, else scs), ``"scs"`` or
            ``"rustypot"``.
        servo_model: Servo model name, used by the rustypot backend.
        baudrate: Bus speed.
        timeout_s: Per-transaction timeout in seconds.
        retries: Extra attempts per transaction.

    Returns:
        A transport implementing :class:`~pysoarm.bus.base.Bus`.

    Raises:
        ConfigurationError: Unknown backend, or ``rustypot`` requested but not installed.
    """
    backend = resolve_backend(backend)
    if backend == "rustypot":
        return RustypotBus(
            port,
            motor_ids,
            servo_model=servo_model,
            baudrate=baudrate,
            timeout_s=timeout_s,
            retries=retries,
        )
    return ScsBus(port, motor_ids, baudrate=baudrate, timeout_s=timeout_s, retries=retries)
