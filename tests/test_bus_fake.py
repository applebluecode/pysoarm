"""The in-memory bus: register semantics, torque, fault injection and call logging."""

from __future__ import annotations

import pytest

from pysoarm.bus.fake import DEFAULT_MODEL_NUMBER, FakeBus
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError


@pytest.fixture
def bus() -> FakeBus:
    bus = FakeBus()
    bus.open()
    return bus


def test_defaults_look_like_a_fresh_arm(bus: FakeBus) -> None:
    assert bus.scan() == dict.fromkeys(bus.motor_ids, DEFAULT_MODEL_NUMBER)
    assert bus.read(1, "present_position") == 2048
    assert bus.read(1, "min_position_limit") == 0
    assert bus.read(1, "max_position_limit") == 4095
    assert bus.read(1, "present_voltage") == 119


def test_goal_is_tracked_only_with_torque(bus: FakeBus) -> None:
    bus.write(1, "goal_position", 3000)
    assert bus.read(1, "present_position") == 3000

    bus.disable_torque()
    bus.write(1, "goal_position", 1000)
    assert bus.read(1, "present_position") == 3000
    assert bus.read(1, "torque_enable") == 0
    assert bus.read(1, "goal_position") == 1000


def test_sync_write_touches_every_motor(bus: FakeBus) -> None:
    bus.sync_write("goal_position", dict.fromkeys(bus.motor_ids, 1500))
    assert bus.sync_read(bus.motor_ids, "present_position") == dict.fromkeys(bus.motor_ids, 1500)


def test_calls_record_one_transaction_per_operation(bus: FakeBus) -> None:
    bus.calls.clear()
    bus.sync_read((1, 2, 3), "present_position")
    bus.sync_write("goal_position", {1: 1000, 2: 1000})
    assert bus.calls == [
        ("sync_read", "present_position", (1, 2, 3)),
        ("sync_write", "goal_position", (1, 2), (1000, 1000)),
    ]


def test_fault_injection_raises_communication_error(bus: FakeBus) -> None:
    bus.fail_next_reads = 1
    with pytest.raises(CommunicationError, match="injected read failure"):
        bus.sync_read((1,), "present_position")
    # the counter is consumed, the next call succeeds
    assert bus.sync_read((1,), "present_position") == {1: 2048}

    bus.fail_next_writes = 1
    with pytest.raises(CommunicationError, match="injected write failure"):
        bus.write(1, "goal_position", 1500)


def test_read_only_and_unknown_registers_are_rejected(bus: FakeBus) -> None:
    with pytest.raises(ConfigurationError, match="read-only"):
        bus.write(1, "present_position", 1)
    with pytest.raises(ConfigurationError, match="Unknown register"):
        bus.read(1, "no_such_register")
    with pytest.raises(ConfigurationError, match="not on this fake bus"):
        bus.read(99, "present_position")


def test_set_register_is_a_test_only_backdoor(bus: FakeBus) -> None:
    bus.set_register(3, "present_temperature", 61)
    assert bus.get_register(3, "present_temperature") == 61


def test_closed_bus_is_reported_as_connection_error() -> None:
    bus = FakeBus()
    with pytest.raises(BusConnectionError, match="not open"):
        bus.read(1, "present_position")
    bus.open()
    assert bus.is_open
    bus.close()
    bus.close()  # idempotent
    assert not bus.is_open
