"""SCS protocol over ``pyserial`` — the reference backend and the only hard dependency.

Wire format (Feetech SCS/STS, 8N1, 1 Mbaud)::

    request   FF FF  ID  LEN  INSTR  [PARAMS...]  CHECKSUM
    response  FF FF  ID  LEN  ERROR  [DATA...]    CHECKSUM

with ``LEN = len(PARAMS) + 2`` (a two-byte READ therefore uses ``LEN = 4`` — the value most
easily got wrong), ``CHECKSUM = ~(ID + LEN + INSTR + PARAMS) & 0xFF`` and broadcast id
``0xFE``. Instructions used here: PING ``0x01``, READ ``0x02``, WRITE ``0x03``,
SYNC_WRITE ``0x83``.

The bus is **half-duplex**: while the host transmits, a servo's reply cannot be on the wire.
Pipelining requests (sending all six before reading any reply) therefore corrupts the stream on
real hardware — measured, and the reason :meth:`ScsBus.sync_read` issues one request/response
pair per servo and collects them into one mapping. Broadcast SYNC_WRITE is the exception: the
host is the only transmitter, so six goals go out in a single frame.

Measured on the reference hardware (six joints, 1 Mbaud, this backend): single register read
0.30 ms, six-joint ``sync_read`` ~1.8 ms (sequential pairs), six-goal ``sync_write`` 0.26 ms.
A Rust driver (``rustypot``) pipelines at a lower level and reaches ~1 ms; this module trades
that for zero non-Python dependencies.
"""

from __future__ import annotations

import contextlib
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Protocol

import serial

from pysoarm.bus.registers import Register, get_register
from pysoarm.constants import BROADCAST_ID, DEFAULT_BAUDRATE, DEFAULT_RETRIES, DEFAULT_TIMEOUT_S
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError

__all__ = ["DEFAULT_PROBE_IDS", "ScsBus", "SerialPort"]

HEADER = b"\xff\xff"
INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_SYNC_WRITE = 0x83
MAX_MOTOR_ID = 253
#: Ids probed by :meth:`ScsBus.scan` when the caller does not specify any.
DEFAULT_PROBE_IDS: tuple[int, ...] = tuple(range(1, 17))
#: Bounded forward scan used to recover frame synchronisation.
_RESYNC_BYTES = 64
#: A status packet always carries at least the error byte and the checksum.
_MIN_STATUS_LEN = 2
#: ``sum(frame) & 0xFF`` of a valid packet, checksum included.
_CHECKSUM_OK = 0xFF
#: Cap on the per-id timeout used by ``scan``, so probing absent ids stays quick.
_SCAN_TIMEOUT_S = 0.01


class SerialPort(Protocol):
    """The subset of ``serial.Serial`` this backend uses (injectable for tests)."""

    @property
    def is_open(self) -> bool:
        """Whether the underlying handle is open."""
        ...

    def write(self, data: bytes) -> int | None:
        """Write bytes to the port."""
        ...

    def read(self, size: int = 1) -> bytes:
        """Read up to ``size`` bytes, honouring the port timeout."""
        ...

    def reset_input_buffer(self) -> None:
        """Discard any buffered input."""
        ...

    def close(self) -> None:
        """Close the port."""
        ...


class ScsBus:
    """SCS/STS register bus over a serial port.

    Args:
        port: Device path, e.g. ``"/dev/ttyACM0"``.
        motor_ids: Ids on this bus; defaults to ``1..6``. Used by ``enable_torque()`` /
            ``disable_torque()`` when no explicit ids are given.
        baudrate: Bus speed; the STS3215 factory default is 1 Mbaud.
        timeout_s: Per-transaction timeout in seconds.
        retries: Extra attempts after a failed transaction.
        serial_factory: Callable returning a :class:`SerialPort`; defaults to
            ``serial.Serial``. Tests inject an in-memory peer here.
    """

    def __init__(
        self,
        port: str,
        motor_ids: Sequence[int] = (1, 2, 3, 4, 5, 6),
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        retries: int = DEFAULT_RETRIES,
        serial_factory: Callable[..., SerialPort] | None = None,
    ) -> None:
        if baudrate <= 0:
            raise ConfigurationError("baudrate must be positive")
        if timeout_s <= 0:
            raise ConfigurationError("timeout_s must be positive")
        if retries < 0:
            raise ConfigurationError("retries must be non-negative")
        self._port = port
        self._motor_ids = tuple(motor_ids)
        self._baudrate = baudrate
        self._timeout_s = timeout_s
        self._retries = retries
        self._serial_factory = serial_factory
        self._ser: SerialPort | None = None

    # -- lifecycle --------------------------------------------------------- #
    @property
    def backend(self) -> str:
        """This transport's name (``"scs"``)."""
        return "scs"

    @property
    def is_open(self) -> bool:
        """Whether the serial port is open."""
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """Open the serial port exclusively.

        Raises:
            BusConnectionError: If the port is already open, missing, or held by another
                process (``Device or resource busy``).
        """
        if self._ser is not None:
            raise BusConnectionError(f"Port {self._port!r} is already open")
        factory = self._serial_factory
        try:
            if factory is None:
                self._ser = serial.Serial(
                    self._port,
                    baudrate=self._baudrate,
                    timeout=self._timeout_s,
                    write_timeout=self._timeout_s,
                    exclusive=True,
                )
            else:
                self._ser = factory(
                    self._port,
                    baudrate=self._baudrate,
                    timeout=self._timeout_s,
                    write_timeout=self._timeout_s,
                    exclusive=True,
                )
        except Exception as exc:  # pyserial raises SerialException/OSError
            self._ser = None
            raise BusConnectionError(
                f"Cannot open {self._port!r}: {exc}. "
                "Check that no other process holds the port (a suspended lerobot-* is a "
                "common cause) and that your user may access the tty."
            ) from exc

    def close(self) -> None:
        """Close the serial port. Idempotent; never raises."""
        ser, self._ser = self._ser, None
        if ser is None:
            return
        with contextlib.suppress(Exception):  # close() must not mask the caller's exception
            ser.close()

    def __enter__(self) -> ScsBus:
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def _require_open(self) -> SerialPort:
        if self._ser is None:
            raise BusConnectionError(f"Port {self._port!r} is not open; call open() first")
        return self._ser

    # -- frame helpers ----------------------------------------------------- #
    @staticmethod
    def _checksum(body: bytes) -> int:
        return ~sum(body) & 0xFF

    @staticmethod
    def _packet(motor_id: int, instruction: int, params: bytes = b"") -> bytes:
        body = bytes([motor_id, len(params) + 2, instruction]) + params
        return HEADER + body + bytes([ScsBus._checksum(body)])

    def _read_exact(self, size: int, deadline: float) -> bytes | None:
        ser = self._require_open()
        buf = b""
        while len(buf) < size:
            if time.monotonic() >= deadline:
                return None
            chunk = ser.read(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _read_packet(self, deadline: float) -> tuple[int, int, bytes] | None:
        """Read one status packet as ``(motor_id, status, data)``."""
        header = self._read_exact(2, deadline)
        if header is None:
            return None
        if header != HEADER:
            tail = header
            for _ in range(_RESYNC_BYTES):
                nxt = self._read_exact(1, deadline)
                if nxt is None:
                    return None
                tail = (tail + nxt)[-2:]
                if tail == HEADER:
                    break
            else:
                raise CommunicationError(
                    f"Lost frame synchronisation (tail={tail!r})", raw=tail, transient=True
                )
        head = self._read_exact(3, deadline)  # id, len, status
        if head is None:
            raise CommunicationError(
                "Truncated status packet (header without body)", transient=True
            )
        motor_id, length, status = head[0], head[1], head[2]
        if length < _MIN_STATUS_LEN:
            raise CommunicationError(
                f"Invalid LEN={length} in status packet from motor {motor_id}",
                motor_id=motor_id,
                raw=head,
                transient=True,
            )
        rest = self._read_exact(length - 1, deadline)  # data + checksum
        if rest is None:
            raise CommunicationError(
                f"Truncated status packet from motor {motor_id}",
                motor_id=motor_id,
                raw=head,
                transient=True,
            )
        data, checksum = rest[:-1], rest[-1]
        body = bytes([motor_id, length, status]) + data
        if (sum(body) + checksum) & 0xFF != _CHECKSUM_OK:
            frame = HEADER + head + rest
            raise CommunicationError(
                f"Checksum mismatch from motor {motor_id}: frame={frame.hex(' ')}",
                motor_id=motor_id,
                raw=frame,
                transient=True,
            )
        return motor_id, status, data

    def _transact(
        self,
        packet: bytes,
        expected_id: int,
        register: str | None = None,
        timeout_s: float | None = None,
    ) -> bytes:
        ser = self._require_open()
        ser.reset_input_buffer()
        ser.write(packet)
        reply = self._read_packet(time.monotonic() + (timeout_s or self._timeout_s))
        if reply is None:
            raise CommunicationError(
                f"No reply from motor {expected_id} ({register or 'request'})",
                motor_id=expected_id,
                register=register,
                raw=packet,
                transient=True,
            )
        motor_id, status, data = reply
        if motor_id != expected_id:
            raise CommunicationError(
                f"Reply from motor {motor_id}, expected {expected_id}",
                motor_id=motor_id,
                register=register,
                raw=data,
                transient=True,
            )
        if status != 0:
            raise CommunicationError(
                f"Motor {motor_id} reported status 0x{status:02x} for {register or 'request'}",
                motor_id=motor_id,
                register=register,
                raw=bytes([status]) + data,
                status=status,
                transient=True,
            )
        return data

    def _with_retries(self, action: Callable[[], bytes], retries: int | None = None) -> bytes:
        """Run ``action`` until it succeeds, honouring the per-call retry override."""
        attempts = (self._retries if retries is None else retries) + 1
        if attempts < 1:
            raise ConfigurationError("retries must be non-negative")
        last: CommunicationError | None = None
        for attempt in range(1, attempts + 1):
            try:
                return action()
            except CommunicationError as exc:
                last = exc
                exc.attempts = attempt
        assert last is not None  # loop runs at least once
        raise last

    # -- register access --------------------------------------------------- #
    @staticmethod
    def _validate_id(motor_id: int) -> int:
        if not 1 <= motor_id <= MAX_MOTOR_ID:
            raise ConfigurationError(f"Motor id must be in 1..{MAX_MOTOR_ID}, got {motor_id}")
        return motor_id

    def _read_register(
        self,
        motor_id: int,
        reg: Register,
        timeout_s: float | None = None,
        retries: int | None = None,
    ) -> int:
        """One request/response pair, with retries."""
        params = bytes([reg.addr, reg.size])
        data = self._with_retries(
            lambda: self._transact(
                self._packet(motor_id, INSTR_READ, params), motor_id, reg.name, timeout_s
            ),
            retries,
        )
        if len(data) != reg.size:
            raise CommunicationError(
                f"Motor {motor_id}: {reg.name} returned {len(data)} byte(s), expected {reg.size}",
                motor_id=motor_id,
                register=reg.name,
                raw=data,
                transient=True,
            )
        return reg.decode(data)

    def read(self, motor_id: int, register: str, *, retries: int | None = None) -> int:
        """Read one register of one motor (see :class:`~pysoarm.bus.base.Bus`)."""
        return self._read_register(
            self._validate_id(motor_id), get_register(register), retries=retries
        )

    def write(
        self,
        motor_id: int,
        register: str,
        value: int,
        *,
        retries: int | None = None,
    ) -> None:
        """Write one register of one motor and wait for the acknowledgement."""
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        mid = self._validate_id(motor_id)
        packet = self._packet(mid, INSTR_WRITE, bytes([reg.addr]) + reg.encode(value))
        self._with_retries(lambda: self._transact(packet, mid, reg.name), retries)

    def sync_read(
        self,
        motor_ids: Sequence[int],
        register: str,
        *,
        retries: int | None = None,
    ) -> dict[int, int]:
        """Read one register from several motors.

        Despite the name this is **not** a broadcast on this hardware: the bus is half-duplex,
        so each request is followed by its reply before the next request is sent. The result is
        a single mapping, which is what the layers above need; the snapshot is skewed by
        ~0.3 ms per servo (documented in :mod:`pysoarm.bus.scs`).
        """
        reg = get_register(register)
        ids = [self._validate_id(mid) for mid in dict.fromkeys(motor_ids)]
        results: dict[int, int] = {}
        for mid in ids:
            try:
                results[mid] = self._read_register(mid, reg, retries=retries)
            except CommunicationError as exc:
                missing = [pending for pending in ids if pending not in results]
                raise CommunicationError(
                    f"No reply from motor(s) {missing} for {reg.name} "
                    f"after {self._retries + 1} attempt(s)",
                    motor_id=mid,
                    register=reg.name,
                    raw=exc.raw,
                    status=exc.status,
                    transient=exc.transient,
                    attempts=exc.attempts,
                ) from exc
        return results

    def sync_write(
        self,
        register: str,
        values: Mapping[int, int],
        *,
        retries: int | None = None,
    ) -> None:
        """Write one register of several motors in a single broadcast frame.

        The frame is fire-and-forget, so ``retries`` only documents intent: the broadcast is
        written once (re-sending it would be idempotent but would not improve reliability).
        """
        del retries
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        if not values:
            return
        payload = b"".join(
            bytes([self._validate_id(mid)]) + reg.encode(value) for mid, value in values.items()
        )
        packet = self._packet(BROADCAST_ID, INSTR_SYNC_WRITE, bytes([reg.addr, reg.size]) + payload)
        self._require_open().write(packet)

    # -- convenience ------------------------------------------------------- #
    def enable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Enable torque on the given ids (default: the bus' motor ids)."""
        ids = self._motor_ids if motor_ids is None else motor_ids
        self.sync_write("torque_enable", dict.fromkeys(ids, 1))

    def disable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Disable torque on the given ids (default: the bus' motor ids)."""
        ids = self._motor_ids if motor_ids is None else motor_ids
        self.sync_write("torque_enable", dict.fromkeys(ids, 0))

    def scan(self, motor_ids: Sequence[int] | None = None) -> dict[int, int]:
        """Probe ids and return ``id -> model number`` for the servos that answered.

        Silent ids are skipped. A short per-id timeout is used, so probing a wide range stays
        fast: an absent id costs ``min(timeout, 10 ms)`` instead of the full transaction timeout.
        """
        ids = list(self._motor_ids if motor_ids is None else motor_ids)
        reg = get_register("model_number")
        timeout = min(self._timeout_s, _SCAN_TIMEOUT_S)
        found: dict[int, int] = {}
        for mid in ids:
            try:
                found[self._validate_id(mid)] = self._read_register(mid, reg, timeout)
            except CommunicationError:
                continue
        return found
