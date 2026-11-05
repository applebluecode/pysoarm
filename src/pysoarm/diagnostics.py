"""Structured motor health: cache refresh, alarm latching and snapshots.

This module owns the *diagnostic view* of the arm: which health registers are cached, how
they are refreshed (all at once or round-robin), how long the cache is valid, and how a
status alarm observed on a failed transaction is turned into :class:`MotorHealth` flags.

It does not own safety decisions. Temperature and voltage are reported here; refusing or
derating a goal remains the job of :mod:`pysoarm.safety` and :class:`~pysoarm.arm.SOArm`.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from typing import cast

from pysoarm.constants import VOLTAGE_LSB_V
from pysoarm.exceptions import CommunicationError
from pysoarm.models.base import RobotModel
from pysoarm.state import MotorHealth

__all__ = ["HEALTH_REGISTERS", "HealthCache", "build_health"]

#: Registers refreshed one per tick when the cache goes stale. Spread over four ticks
#: (~+0.36 ms each) so no single control tick pays for a full health read.
HEALTH_REGISTERS: tuple[str, ...] = (
    "present_speed",
    "present_current",
    "present_temperature",
    "present_voltage",
)

#: A read function as used by :class:`HealthCache` (``SOArm._read`` is the usual binding).
ReadRegister = Callable[[str, "int | None"], Mapping[int, int]]


class HealthCache:
    """Cached health registers with an incremental refresh policy.

    Args:
        motor_ids: Servo ids in the model's joint order.
        clock: Monotonic clock used for cache-age accounting.
    """

    def __init__(self, motor_ids: Sequence[int], clock: Callable[[], float]) -> None:
        self._ids = tuple(motor_ids)
        self._clock = clock
        self._values: dict[str, tuple[float, ...] | tuple[int, ...]] = {}
        self._flags: dict[int, set[str]] = {}
        self._index = 0
        self._refreshed_at = -math.inf

    # -- cache state ------------------------------------------------------- #
    @property
    def refreshed_at(self) -> float:
        """Monotonic time of the last completed refresh (``-inf`` before the first)."""
        return self._refreshed_at

    def age(self, now: float) -> float:
        """Age of the cache in seconds (``inf`` when it was never refreshed)."""
        return max(0.0, now - self._refreshed_at) if math.isfinite(self._refreshed_at) else math.inf

    def flags_for(self, motor_id: int) -> frozenset[str]:
        """Alarm bits latched for one motor."""
        return frozenset(self._flags.get(motor_id, set()))

    def latch(self, error: CommunicationError) -> None:
        """Remember the named alarm bits carried by a failed transaction."""
        if error.motor_id is None or not error.flags:
            return
        self._flags.setdefault(error.motor_id, set()).update(error.flags)

    def _zeros_int(self) -> tuple[int, ...]:
        return (0,) * len(self._ids)

    @property
    def speed_raw(self) -> tuple[int, ...]:
        """Cached raw ``present_speed`` values (zeros before the first refresh)."""
        value = self._values.get("speed_raw")
        return (
            cast("tuple[int, ...]", value)
            if isinstance(value, tuple) and value and isinstance(value[0], int)
            else self._zeros_int()
        )

    @property
    def current(self) -> tuple[int, ...]:
        """Cached raw ``present_current`` values (zeros before the first refresh)."""
        value = self._values.get("current")
        return (
            cast("tuple[int, ...]", value)
            if isinstance(value, tuple) and value and isinstance(value[0], int)
            else self._zeros_int()
        )

    @property
    def temperature_c(self) -> tuple[int, ...]:
        """Cached motor temperatures (zeros before the first refresh)."""
        value = self._values.get("temperature_c")
        return (
            cast("tuple[int, ...]", value)
            if isinstance(value, tuple) and value and isinstance(value[0], int)
            else self._zeros_int()
        )

    @property
    def voltage_v(self) -> tuple[float, ...]:
        """Cached supply voltages in volts (zeros before the first refresh)."""
        value = self._values.get("voltage_v")
        return (
            cast("tuple[float, ...]", value)
            if isinstance(value, tuple) and value and isinstance(value[0], float)
            else (0.0,) * len(self._ids)
        )

    # -- refresh ----------------------------------------------------------- #
    def _store(self, register: str, values: Mapping[int, int]) -> None:
        ids = self._ids
        if register == "present_voltage":
            self._values["voltage_v"] = tuple(values[mid] * VOLTAGE_LSB_V for mid in ids)
        elif register == "present_temperature":
            self._values["temperature_c"] = tuple(values[mid] for mid in ids)
        elif register == "present_speed":
            self._values["speed_raw"] = tuple(values[mid] for mid in ids)
        else:
            self._values["current"] = tuple(values[mid] for mid in ids)

    def refresh_all(self, read: ReadRegister, retries: int | None = None) -> None:
        """Read every health register and mark the cache fresh."""
        for register in HEALTH_REGISTERS:
            self._store(register, read(register, retries))
        self._index = 0
        self._refreshed_at = self._clock()

    def refresh_incremental(self, read: ReadRegister, retries: int | None = None) -> None:
        """Read one health register; the cache counts as fresh after four ticks."""
        register = HEALTH_REGISTERS[self._index % len(HEALTH_REGISTERS)]
        self._store(register, read(register, retries))
        self._index += 1
        if self._index % len(HEALTH_REGISTERS) == 0:
            self._refreshed_at = self._clock()


def build_health(
    model: RobotModel,
    cache: HealthCache,
    *,
    temp_warn_c: float,
    voltage_min_v: float,
) -> dict[str, MotorHealth]:
    """Assemble a structured health view from a refreshed cache.

    Args:
        model: Arm model defining joint names and motor ids.
        cache: Already-refreshed health cache.
        temp_warn_c: Temperature at or above which ``"overheat"`` is reported.
        voltage_min_v: Supply voltage below which ``"voltage"`` is reported.

    Returns:
        Mapping ``joint -> MotorHealth`` in the model's joint order.
    """
    voltage = cache.voltage_v
    temperature = cache.temperature_c
    current = cache.current
    health: dict[str, MotorHealth] = {}
    for index, (joint, mid) in enumerate(zip(model.joints, model.motor_ids, strict=True)):
        flags = set(cache.flags_for(mid))
        volts = voltage[index] if index < len(voltage) else 0.0
        temp = temperature[index] if index < len(temperature) else 0
        if volts and volts < voltage_min_v:
            flags.add("voltage")
        if temp >= temp_warn_c:
            flags.add("overheat")
        health[joint] = MotorHealth(
            joint=joint,
            motor_id=mid,
            voltage_v=volts,
            temperature_c=temp,
            current_raw=current[index] if index < len(current) else 0,
            flags=frozenset(flags),
        )
    return health
