"""Deterministic in-memory bus for tests and offline development.

The fake keeps a register map per motor, moves ``present_position`` to ``goal_position`` when a
goal is written (instant tracking, torque on), and records every transaction in
:attr:`FakeBus.calls` so tests can assert that the control loop issues exactly one sync read
and one sync write per tick — never a per-joint loop.

Faults are injectable: :attr:`fail_next_reads` / :attr:`fail_next_writes` make the next
transactions raise :class:`~pysoarm.exceptions.CommunicationError`, and ``latency_s`` adds a
sleep to every transaction so overrun handling can be exercised without hardware.
"""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence

from pysoarm.bus.registers import get_register
from pysoarm.constants import VOLTAGE_LSB_V
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError

__all__ = ["DEFAULT_MODEL_NUMBER", "FakeBus"]

#: STS3215 model number as reported by the ``model_number`` register.
DEFAULT_MODEL_NUMBER = 777


class FakeBus:
    """In-memory stand-in for :class:`~pysoarm.bus.scs.ScsBus`.

    Args:
        motor_ids: Ids present on the fake bus.
        model_number: Value returned by ``scan`` and the ``model_number`` register.
        initial_position: Starting ``present_position`` for every motor (raw ticks).
        torque_enabled: Initial ``torque_enable`` state.
        temperature_c: Plausible motor temperature, used by the safety guards.
        voltage_v: Bus supply voltage in volts.
        current: ``present_current`` value (servo counts, not amperes).
        latency_s: Sleep injected into every transaction, to simulate a slow bus.
        instant_tracking: When ``True`` (default) writing a goal moves ``present_position``
            to the goal immediately, so tests are deterministic.
    """

    def __init__(
        self,
        motor_ids: Sequence[int] = (1, 2, 3, 4, 5, 6),
        model_number: int = DEFAULT_MODEL_NUMBER,
        initial_position: int = 2048,
        *,
        torque_enabled: bool = True,
        temperature_c: int = 35,
        voltage_v: float = 11.9,
        current: int = 0,
        latency_s: float = 0.0,
        instant_tracking: bool = True,
    ) -> None:
        self.motor_ids: tuple[int, ...] = tuple(motor_ids)
        self.model_number = model_number
        self.latency_s = latency_s
        self.instant_tracking = instant_tracking
        self.calls: list[tuple[object, ...]] = []
        self.fail_next_reads = 0
        self.fail_next_writes = 0
        self._is_open = False
        self._regs: dict[int, dict[str, int]] = {
            mid: self._default_registers(
                model_number=model_number,
                position=initial_position,
                torque_enabled=torque_enabled,
                temperature_c=temperature_c,
                voltage_v=voltage_v,
                current=current,
            )
            for mid in self.motor_ids
        }

    @staticmethod
    def _default_registers(
        *,
        model_number: int,
        position: int,
        torque_enabled: bool,
        temperature_c: int,
        voltage_v: float,
        current: int,
    ) -> dict[str, int]:
        return {
            "model_number": model_number,
            "return_delay_time": 0,
            "min_position_limit": 0,
            "max_position_limit": 4095,
            "max_torque_limit": 1000,
            "torque_limit": 1000,
            "phase": 0,
            "p_coefficient": 16,
            "d_coefficient": 32,
            "i_coefficient": 0,
            "protection_current": 500,
            "homing_offset": 0,
            "operating_mode": 0,
            "overload_torque": 50,
            "torque_enable": 1 if torque_enabled else 0,
            "acceleration": 254,
            "goal_speed": 0,
            "goal_position": position,
            "maximum_acceleration": 254,
            "present_position": position,
            "present_speed": 0,
            "present_load": 0,
            "present_voltage": round(voltage_v / VOLTAGE_LSB_V),
            "present_temperature": temperature_c,
            "present_current": current,
        }

    # -- test helpers ------------------------------------------------------ #
    def set_register(self, motor_id: int, register: str, value: int) -> None:
        """Overwrite a register without going through the protocol (test setup)."""
        if motor_id not in self._regs:
            raise ConfigurationError(f"Motor {motor_id} is not on this fake bus")
        get_register(register)  # validate the name
        self._regs[motor_id][register] = value

    def get_register(self, motor_id: int, register: str) -> int:
        """Read a register directly, bypassing the protocol (test assertions)."""
        return self._regs[motor_id][register]

    def _maybe_fail(self, kind: str) -> None:
        if self.latency_s:
            time.sleep(self.latency_s)
        if kind == "read" and self.fail_next_reads > 0:
            self.fail_next_reads -= 1
            raise CommunicationError("injected read failure", raw=b"")
        if kind == "write" and self.fail_next_writes > 0:
            self.fail_next_writes -= 1
            raise CommunicationError("injected write failure", raw=b"")

    def _require_open(self) -> None:
        if not self._is_open:
            raise BusConnectionError("fake bus is not open")

    def _validate(self, motor_id: int) -> dict[str, int]:
        try:
            return self._regs[motor_id]
        except KeyError as exc:
            raise ConfigurationError(f"Motor {motor_id} is not on this fake bus") from exc

    # -- Bus protocol ------------------------------------------------------ #
    @property
    def backend(self) -> str:
        """This transport's name (``"fake"``)."""
        return "fake"

    @property
    def is_open(self) -> bool:
        """Whether the fake bus is open."""
        return self._is_open

    def open(self) -> None:
        """Open the fake bus."""
        self._is_open = True

    def close(self) -> None:
        """Close the fake bus (idempotent)."""
        self._is_open = False

    def __enter__(self) -> FakeBus:
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def read(self, motor_id: int, register: str, *, retries: int | None = None) -> int:
        """Read one register of one motor."""
        del retries  # the fake never retries: failures are injected deterministically
        reg = get_register(register)
        self._require_open()
        self._maybe_fail("read")
        self.calls.append(("read", reg.name, motor_id))
        regs = self._validate(motor_id)
        try:
            return regs[reg.name]
        except KeyError as exc:
            raise ConfigurationError(
                f"Motor {motor_id} does not expose register {reg.name!r}"
            ) from exc

    def write(
        self, motor_id: int, register: str, value: int, *, retries: int | None = None
    ) -> None:
        """Write one register of one motor."""
        del retries
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        self._require_open()
        self._maybe_fail("write")
        self.calls.append(("write", reg.name, motor_id, value))
        self._apply(motor_id, reg.name, value)

    def sync_read(
        self, motor_ids: Sequence[int], register: str, *, retries: int | None = None
    ) -> dict[int, int]:
        """Read one register from several motors in one call."""
        del retries
        reg = get_register(register)
        self._require_open()
        self._maybe_fail("read")
        ids = tuple(dict.fromkeys(motor_ids))
        self.calls.append(("sync_read", reg.name, ids))
        result: dict[int, int] = {}
        for mid in ids:
            try:
                result[mid] = self._validate(mid)[reg.name]
            except KeyError as exc:
                raise ConfigurationError(
                    f"Motor {mid} does not expose register {reg.name!r}"
                ) from exc
        return result

    def sync_write(
        self,
        register: str,
        values: Mapping[int, int],
        *,
        retries: int | None = None,
    ) -> None:
        """Write one register of several motors in one call."""
        del retries
        reg = get_register(register)
        if not reg.writable:
            raise ConfigurationError(f"Register {reg.name!r} is read-only")
        self._require_open()
        self._maybe_fail("write")
        self.calls.append(("sync_write", reg.name, tuple(values), tuple(values.values())))
        for mid, value in values.items():
            self._apply(mid, reg.name, value)

    def _apply(self, motor_id: int, register: str, value: int) -> None:
        regs = self._validate(motor_id)
        regs[register] = value
        if (
            register == "goal_position"
            and self.instant_tracking
            and regs.get("torque_enable", 0) == 1
        ):
            regs["present_position"] = value

    def enable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Enable torque on the given ids (default: all)."""
        ids = self.motor_ids if motor_ids is None else motor_ids
        self.sync_write("torque_enable", dict.fromkeys(ids, 1))

    def disable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Disable torque on the given ids (default: all)."""
        ids = self.motor_ids if motor_ids is None else motor_ids
        self.sync_write("torque_enable", dict.fromkeys(ids, 0))

    def scan(self, motor_ids: Sequence[int] | None = None) -> dict[int, int]:
        """Return ``id -> model_number`` for the ids that exist on this fake bus."""
        self._require_open()
        ids = self.motor_ids if motor_ids is None else tuple(motor_ids)
        self.calls.append(("scan", tuple(ids)))
        return {mid: self._regs[mid]["model_number"] for mid in ids if mid in self._regs}
