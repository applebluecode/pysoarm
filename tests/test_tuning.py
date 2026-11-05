"""Tuning-register helpers used by SOArm.motor_config/set_motor_config."""

from __future__ import annotations

import pytest

from pysoarm.bus.fake import FakeBus
from pysoarm.exceptions import ConfigurationError
from pysoarm.models import SO101
from pysoarm.tuning import MOTOR_CONFIG_REGISTERS, read_motor_config, write_motor_config


@pytest.fixture
def bus() -> FakeBus:
    bus = FakeBus()
    bus.open()
    return bus


def test_read_motor_config_returns_every_supported_register(bus: FakeBus) -> None:
    config = read_motor_config(bus, SO101, "gripper")
    assert set(config) == set(MOTOR_CONFIG_REGISTERS)
    assert config["protection_current"] == 500
    assert config["acceleration"] == 254


def test_write_motor_config_updates_the_registers(bus: FakeBus) -> None:
    written = write_motor_config(bus, SO101, "gripper", {"acceleration": 120, "p_coefficient": 10})
    assert written == {"acceleration": 120, "p_coefficient": 10}
    assert bus.get_register(6, "acceleration") == 120
    assert bus.get_register(6, "p_coefficient") == 10


def test_unknown_register_is_rejected_before_touching_the_bus(bus: FakeBus) -> None:
    with pytest.raises(ConfigurationError, match="Unknown motor config"):
        write_motor_config(bus, SO101, "gripper", {"no_such_register": 1})
    assert bus.get_register(6, "acceleration") == 254


def test_unknown_joint_is_rejected(bus: FakeBus) -> None:
    with pytest.raises(ConfigurationError, match="Unknown joint"):
        read_motor_config(bus, SO101, "no_such_joint")
