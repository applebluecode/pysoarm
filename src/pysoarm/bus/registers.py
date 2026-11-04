"""STS3215 register table and value encodings.

Addresses, widths and encodings follow the Feetech STS3215 memory table (the same values the
LeRobot ecosystem writes), cross-checked against the definitions shipped with ``rustypot``.

Positions, speeds, loads and homing offsets use **sign-magnitude**, not two's complement:
the sign lives in ``sign_bit`` and the remaining bits carry the magnitude. Decoding them as
two's complement is the classic bug this module exists to prevent.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from pysoarm.constants import VOLTAGE_LSB_V
from pysoarm.exceptions import ConfigurationError

__all__ = ["REGISTERS", "Register", "RegisterKind", "get_register"]


class RegisterKind(str, Enum):
    """How the raw bytes of a register map to an integer."""

    UNSIGNED = "unsigned"
    SIGN_MAGNITUDE = "sign_magnitude"


@dataclass(frozen=True, slots=True)
class Register:
    """One STS3215 register.

    Args:
        name: Canonical snake_case name, e.g. ``"present_position"``.
        addr: Memory-table address.
        size: Width in bytes (1 or 2).
        kind: integer encoding.
        sign_bit: Bit carrying the sign for sign-magnitude registers (``None`` otherwise).
        scale: Multiply the decoded integer by this to get engineering units.
        unit: Engineering unit label (``"deg"``, ``"C"``, ``"V"``, ``""``, ...).
        writable: Whether the register accepts writes.
    """

    name: str
    addr: int
    size: int
    kind: RegisterKind = RegisterKind.UNSIGNED
    sign_bit: int | None = None
    scale: float = 1.0
    unit: str = ""
    writable: bool = True

    def __post_init__(self) -> None:
        if self.size not in (1, 2):
            raise ConfigurationError(f"Register {self.name!r}: size must be 1 or 2 bytes")
        if self.kind is RegisterKind.SIGN_MAGNITUDE:
            if self.sign_bit is None or not 0 <= self.sign_bit < self.size * 8:
                raise ConfigurationError(
                    f"Register {self.name!r}: sign-magnitude needs a valid sign_bit"
                )
        elif self.sign_bit is not None:
            raise ConfigurationError(f"Register {self.name!r}: sign_bit requires sign_magnitude")

    # -- encoding ---------------------------------------------------------- #
    def decode(self, data: bytes) -> int:
        """Decode raw little-endian bytes into an integer.

        Args:
            data: Exactly ``size`` bytes as received on the bus.

        Returns:
            The signed or unsigned integer value.

        Raises:
            ConfigurationError: If ``data`` has the wrong length.
        """
        if len(data) != self.size:
            raise ConfigurationError(
                f"Register {self.name!r}: expected {self.size} byte(s), got {len(data)}"
            )
        value = int.from_bytes(data, "little")
        if self.kind is RegisterKind.SIGN_MAGNITUDE and self.sign_bit is not None:
            sign_mask = 1 << self.sign_bit
            if value & sign_mask:
                return -(value & (sign_mask - 1))
        return value

    def encode(self, value: int) -> bytes:
        """Encode an integer into raw little-endian bytes.

        Args:
            value: Value to encode; clamped to the register's representable magnitude.

        Returns:
            Exactly ``size`` bytes.
        """
        if self.kind is RegisterKind.SIGN_MAGNITUDE and self.sign_bit is not None:
            sign_mask = 1 << self.sign_bit
            magnitude = min(abs(value), sign_mask - 1)
            raw = magnitude | (sign_mask if value < 0 else 0)
        else:
            raw = int(value)
        raw &= (1 << (self.size * 8)) - 1
        return raw.to_bytes(self.size, "little")

    # -- engineering units ------------------------------------------------- #
    def to_unit(self, raw: int) -> float:
        """Convert a decoded integer to engineering units."""
        return raw * self.scale

    def from_unit(self, value: float) -> int:
        """Convert engineering units to the nearest representable integer."""
        return round(value / self.scale)

    def __str__(self) -> str:
        return f"{self.name}@{self.addr}"


def _reg(
    name: str,
    addr: int,
    size: int,
    *,
    sign_bit: int | None = None,
    scale: float = 1.0,
    unit: str = "",
    writable: bool = True,
) -> Register:
    kind = RegisterKind.SIGN_MAGNITUDE if sign_bit is not None else RegisterKind.UNSIGNED
    return Register(
        name=name,
        addr=addr,
        size=size,
        kind=kind,
        sign_bit=sign_bit,
        scale=scale,
        unit=unit,
        writable=writable,
    )


#: Registers used by pysoarm. Ordering follows the memory table, not importance.
REGISTERS: dict[str, Register] = {
    # identification / configuration
    "model_number": _reg("model_number", 3, 2, writable=False),
    "id": _reg("id", 5, 1),
    "baudrate": _reg("baudrate", 6, 1),
    "return_delay_time": _reg("return_delay_time", 7, 1),
    "min_position_limit": _reg("min_position_limit", 9, 2),
    "max_position_limit": _reg("max_position_limit", 11, 2),
    "max_torque_limit": _reg("max_torque_limit", 16, 2),
    "phase": _reg("phase", 18, 1),
    "p_coefficient": _reg("p_coefficient", 21, 1),
    "d_coefficient": _reg("d_coefficient", 22, 1),
    "i_coefficient": _reg("i_coefficient", 23, 1),
    "protection_current": _reg("protection_current", 28, 2),
    "homing_offset": _reg("homing_offset", 31, 2, sign_bit=11),
    "operating_mode": _reg("operating_mode", 33, 1),
    "overload_torque": _reg("overload_torque", 36, 1),
    # commands
    "torque_enable": _reg("torque_enable", 40, 1),
    "acceleration": _reg("acceleration", 41, 1),
    "goal_position": _reg("goal_position", 42, 2, sign_bit=15),
    "goal_time": _reg("goal_time", 44, 2),
    "goal_speed": _reg("goal_speed", 46, 2, sign_bit=15),
    "torque_limit": _reg("torque_limit", 48, 2),
    # feedback
    "present_position": _reg("present_position", 56, 2, sign_bit=15, writable=False),
    "present_speed": _reg("present_speed", 58, 2, sign_bit=15, writable=False),
    "present_load": _reg("present_load", 60, 2, sign_bit=10, writable=False),
    "present_voltage": _reg(
        "present_voltage", 62, 1, scale=VOLTAGE_LSB_V, unit="V", writable=False
    ),
    "present_temperature": _reg("present_temperature", 63, 1, unit="C", writable=False),
    "present_current": _reg("present_current", 69, 2, writable=False),
    "maximum_acceleration": _reg("maximum_acceleration", 85, 1),
}


def get_register(name: str) -> Register:
    """Look up a register by name.

    Args:
        name: Register name, case-insensitive.

    Returns:
        The matching register.

    Raises:
        ConfigurationError: If the register is unknown.
    """
    try:
        return REGISTERS[name.lower()]
    except KeyError as exc:
        known = ", ".join(sorted(REGISTERS))
        raise ConfigurationError(f"Unknown register {name!r}. Known: {known}") from exc
