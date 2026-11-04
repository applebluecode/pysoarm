"""Optional ``rustypot`` backend: the same bus, driven from Rust.

``rustypot`` is the driver LeRobot uses for the STS3215. It speaks the same SCS protocol but
does the request/response bookkeeping in Rust, so a six-joint read costs ~1 ms instead of the
~2.2 ms the pure-Python :class:`~pysoarm.bus.scs.ScsBus` needs. It is an **optional** extra::

    pip install "pysoarm[rustypot]"

Select it explicitly — ``SOArm(..., backend="rustypot")``, ``pysoarm scan --backend rustypot`` —
or pass ``backend="auto"`` to use it when it is importable and fall back to ``scs`` otherwise.

Notes on the mapping:

* ``rustypot`` does not reject writes to read-only registers, so this adapter checks
  :data:`~pysoarm.bus.registers.REGISTERS` first (same behaviour as ``ScsBus``);
* rustypot raises ``ValueError`` for a bad id/register name, ``RuntimeError`` for a bus
  failure and ``OSError`` when the port cannot be opened; these are translated to
  :class:`~pysoarm.exceptions.ConfigurationError`, ``CommunicationError`` and
  ``BusConnectionError`` respectively;
* the port is opened with an exclusive lock, so a second process fails loudly.
"""

from __future__ import annotations

import contextlib
import importlib
from collections.abc import Callable, Mapping, Sequence
from typing import Any, TypeVar

from pysoarm.bus.registers import get_register
from pysoarm.constants import DEFAULT_BAUDRATE, DEFAULT_RETRIES, DEFAULT_TIMEOUT_S
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError

__all__ = ["RustypotBus", "require_rustypot"]

_INSTALL_HINT = 'pip install "pysoarm[rustypot]"'
_T = TypeVar("_T")


def _import_rustypot() -> Any:
    """Import the optional dependency (a seam, so tests can simulate it being absent)."""
    return importlib.import_module("rustypot")


def require_rustypot(module: Any | None = None) -> Any:
    """Return the ``rustypot`` module.

    Args:
        module: Injection seam for tests; when given it is returned unchanged.

    Raises:
        ConfigurationError: If ``rustypot`` is not installed, with the install command.
    """
    if module is not None:
        return module
    try:
        return _import_rustypot()
    except ModuleNotFoundError as exc:
        raise ConfigurationError(
            "The rustypot backend needs the optional 'rustypot' dependency; "
            f"install it with {_INSTALL_HINT}."
        ) from exc


class RustypotBus:
    """``Bus`` implementation on top of ``rustypot.Bus``.

    Args:
        port: Serial device, e.g. ``"/dev/so101_follower"``.
        motor_ids: Servo ids on this bus; defaults to ``1..6``.
        servo_model: rustypot servo model name, ``"sts3215"`` for SO-100/SO-101.
        baudrate: Bus speed; the STS3215 factory default is 1 Mbaud.
        timeout_s: Per-transaction timeout in seconds.
        retries: Extra attempts per transaction (handled inside rustypot).
        rustypot_module: Injection seam for tests; defaults to importing ``rustypot``.

    Raises:
        ConfigurationError: If ``rustypot`` is missing or does not know ``servo_model``.
    """

    def __init__(
        self,
        port: str,
        motor_ids: Sequence[int] = (1, 2, 3, 4, 5, 6),
        *,
        servo_model: str = "sts3215",
        baudrate: int = DEFAULT_BAUDRATE,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        retries: int = DEFAULT_RETRIES,
        rustypot_module: Any | None = None,
    ) -> None:
        if baudrate <= 0:
            raise ConfigurationError("baudrate must be positive")
        if timeout_s <= 0:
            raise ConfigurationError("timeout_s must be positive")
        if retries < 0:
            raise ConfigurationError("retries must be non-negative")
        self._rp = require_rustypot(rustypot_module)
        found = self._rp.find_model(servo_model)
        if found is None:
            raise ConfigurationError(f"rustypot has no servo model named {servo_model!r}")
        self._model_number, self._definition = found
        self._port = port
        self._motor_ids = tuple(motor_ids)
        self._baudrate = baudrate
        self._timeout_s = timeout_s
        self._retries = retries
        self._definitions: dict[int, Any] = dict.fromkeys(self._motor_ids, self._definition)
        self._bus: Any | None = None

    # -- lifecycle --------------------------------------------------------- #
    @property
    def is_open(self) -> bool:
        """Whether the rustypot bus is open."""
        return self._bus is not None and bool(self._bus.is_open)

    def open(self) -> None:
        """Open the port with an exclusive lock.

        Raises:
            BusConnectionError: If the port is already open, missing or held by another process.
        """
        if self._bus is not None:
            raise BusConnectionError(f"Port {self._port!r} is already open")
        try:
            self._bus = self._rp.Bus(self._port, self._baudrate, self._timeout_s, self._definitions)
        except OSError as exc:
            self._bus = None
            raise BusConnectionError(
                f"Cannot open {self._port!r} with rustypot: {exc}. "
                "Check that no other process holds the port (a suspended lerobot-* is a "
                "common cause) and that your user may access the tty."
            ) from exc

    def close(self) -> None:
        """Close the bus. Idempotent; never raises."""
        bus, self._bus = self._bus, None
        if bus is None:
            return
        with contextlib.suppress(Exception):  # close() must not mask the caller's exception
            bus.close()

    def __enter__(self) -> RustypotBus:
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- helpers ----------------------------------------------------------- #
    def _require_bus(self) -> Any:
        if self._bus is None:
            raise BusConnectionError(f"Port {self._port!r} is not open; call open() first")
        return self._bus

    def _validate_id(self, motor_id: int) -> int:
        if motor_id not in self._definitions:
            raise ConfigurationError(
                f"Motor {motor_id} is not part of this bus (ids: {sorted(self._definitions)})"
            )
        return motor_id

    def _call(
        self,
        action: Callable[[], _T],
        *,
        motor_id: int | None = None,
        register: str | None = None,
    ) -> _T:
        try:
            return action()
        except ValueError as exc:  # rustypot: bad id or register name
            raise ConfigurationError(f"rustypot: {exc}") from exc
        except (RuntimeError, OSError) as exc:  # timeout / bus failure
            raise CommunicationError(
                f"rustypot: {exc}", motor_id=motor_id, register=register
            ) from exc

    def _retrying(
        self,
        action: Callable[[], _T],
        *,
        motor_id: int | None = None,
        register: str | None = None,
    ) -> _T:
        """Call rustypot with rustypot's own retries disabled, retrying here instead.

        Retrying at this layer (not inside rustypot) keeps the semantics identical to
        :class:`~pysoarm.bus.scs.ScsBus`, including for intermittent parse errors, and avoids
        squaring the attempt count. Status-packet alarms are *not* retried by callers.
        """
        last: CommunicationError | None = None
        for _ in range(self._retries + 1):
            try:
                return self._call(action, motor_id=motor_id, register=register)
            except CommunicationError as exc:
                last = exc
        assert last is not None  # the loop runs at least once
        raise last

    # -- register access --------------------------------------------------- #
    def read(self, motor_id: int, register: str) -> int:
        """Read one register of one motor."""
        reg = get_register(register)
        mid = self._validate_id(motor_id)
        bus = self._require_bus()
        value, status = self._retrying(
            lambda: bus.read_register_with_error(mid, reg.name, retries=0),
            motor_id=mid,
            register=reg.name,
        )
        if status:
            raise CommunicationError(
                f"Motor {mid} reported status 0x{status:02x} for {reg.name}",
                motor_id=mid,
                register=reg.name,
            )
        return int(value)

    def write(self, motor_id: int, register: str, value: int) -> None:
        """Write one register of one motor."""
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        mid = self._validate_id(motor_id)
        bus = self._require_bus()
        self._retrying(
            lambda: bus.write_register(mid, reg.name, int(value), retries=0),
            motor_id=mid,
            register=reg.name,
        )

    def sync_read(self, motor_ids: Sequence[int], register: str) -> dict[int, int]:
        """Read one register from several motors in one rustypot batch."""
        reg = get_register(register)
        ids = [self._validate_id(mid) for mid in dict.fromkeys(motor_ids)]
        if not ids:
            return {}
        bus = self._require_bus()
        pairs = self._retrying(
            lambda: bus.sync_read_register_with_error(ids, reg.name, retries=0),
            register=reg.name,
        )
        results: dict[int, int] = {}
        for mid, (value, status) in zip(ids, pairs, strict=True):
            if status:
                raise CommunicationError(
                    f"Motor {mid} reported status 0x{status:02x} for {reg.name}",
                    motor_id=mid,
                    register=reg.name,
                )
            results[mid] = int(value)
        return results

    def sync_write(self, register: str, values: Mapping[int, int]) -> None:
        """Write one register of several motors in one rustypot sync-write group."""
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        if not values:
            return
        ids = [self._validate_id(mid) for mid in values]
        bus = self._require_bus()
        self._retrying(
            lambda: bus.sync_write_register(
                ids, reg.name, [int(v) for v in values.values()], retries=0
            ),
            register=reg.name,
        )

    # -- convenience ------------------------------------------------------- #
    def enable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Enable torque on the given ids (default: the bus' motor ids)."""
        self._set_torque(motor_ids, enabled=True)

    def disable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Disable torque on the given ids (default: the bus' motor ids)."""
        self._set_torque(motor_ids, enabled=False)

    def _set_torque(self, motor_ids: Sequence[int] | None, *, enabled: bool) -> None:
        selected = self._motor_ids if motor_ids is None else motor_ids
        ids = [self._validate_id(mid) for mid in selected]
        bus = self._require_bus()
        failed = self._retrying(lambda: bus.set_torque(ids, enabled, retries=0))
        if failed:
            details = ", ".join(f"{mid}: {error}" for mid, error in failed.items())
            raise CommunicationError(
                f"Failed to {'enable' if enabled else 'disable'} torque on {details}"
            )

    def scan(self, motor_ids: Sequence[int] | None = None) -> dict[int, int]:
        """Probe ids and return ``id -> model number`` for the servos that answered."""
        ids = list(self._motor_ids if motor_ids is None else motor_ids)
        bus = self._require_bus()
        found = self._call(lambda: bus.scan(self._definition, ids))
        return {int(mid): int(model) for mid, model in found.items()}
