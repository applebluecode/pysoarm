"""STS3215 register table: addresses, widths, sign-magnitude encoding and units."""

from __future__ import annotations

import pytest

from pysoarm.bus.registers import REGISTERS, Register, RegisterKind, get_register
from pysoarm.exceptions import ConfigurationError


def test_key_addresses_match_the_memory_table() -> None:
    expected = {
        "model_number": 3,
        "min_position_limit": 9,
        "max_position_limit": 11,
        "homing_offset": 31,
        "operating_mode": 33,
        "torque_enable": 40,
        "goal_position": 42,
        "present_position": 56,
        "present_speed": 58,
        "present_load": 60,
        "present_voltage": 62,
        "present_temperature": 63,
        "present_current": 69,
        "maximum_acceleration": 85,
    }
    for name, addr in expected.items():
        assert REGISTERS[name].addr == addr, name


def test_positions_use_sign_magnitude_not_twos_complement() -> None:
    position = get_register("present_position")
    assert position.kind is RegisterKind.SIGN_MAGNITUDE
    assert position.sign_bit == 15
    # full-scale positive value fits in 12 bits, so the sign bit stays clear
    assert position.encode(4094) == bytes([0xFE, 0x0F])
    # 0x8010 is -16 in sign-magnitude; as two's complement it would be -32752
    assert position.decode(bytes([0x10, 0x80])) == -16
    assert position.decode(bytes([0xFF, 0x0F])) == 4095


def test_load_uses_bit_ten_as_sign() -> None:
    load = get_register("present_load")
    assert load.sign_bit == 10
    assert load.decode(bytes([0x05, 0x04])) == -5
    assert load.decode(bytes([0x05, 0x00])) == 5


@pytest.mark.parametrize("value", [-2047, -1, 0, 1, 2047, 4095])
def test_sign_magnitude_round_trip(value: int) -> None:
    register = get_register("goal_position")
    assert register.decode(register.encode(value)) == value


def test_unsigned_register_round_trip() -> None:
    limits = get_register("max_position_limit")
    assert limits.kind is RegisterKind.UNSIGNED
    assert limits.encode(4095) == bytes([0xFF, 0x0F])
    assert limits.decode(bytes([0xFF, 0x0F])) == 4095


def test_voltage_scales_to_volts() -> None:
    voltage = get_register("present_voltage")
    assert voltage.scale == pytest.approx(0.1)
    assert voltage.to_unit(119) == pytest.approx(11.9)
    assert voltage.from_unit(11.9) == 119


def test_unknown_register_raises_configuration_error() -> None:
    with pytest.raises(ConfigurationError, match="Unknown register"):
        get_register("does_not_exist")


def test_read_only_registers_are_flagged() -> None:
    assert not get_register("present_position").writable
    assert get_register("goal_position").writable


def test_names_are_case_insensitive() -> None:
    assert get_register("Present_Position") is get_register("present_position")


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"size": 3}, "size must be 1 or 2"),
        ({"kind": RegisterKind.SIGN_MAGNITUDE}, "valid sign_bit"),
        ({"sign_bit": 7}, "requires sign_magnitude"),
    ],
)
def test_invalid_register_definitions_are_rejected(kwargs: dict[str, object], match: str) -> None:
    base: dict[str, object] = {"name": "x", "addr": 1, "size": 2}
    with pytest.raises(ConfigurationError, match=match):
        Register(**{**base, **kwargs})  # type: ignore[arg-type]


def test_decode_rejects_wrong_width() -> None:
    with pytest.raises(ConfigurationError, match="expected 2 byte"):
        get_register("goal_position").decode(b"\x01")
