"""Health cache and structured motor health, without an SOArm."""

from __future__ import annotations

import math
from collections.abc import Callable

import pytest

from pysoarm.diagnostics import HEALTH_REGISTERS, HealthCache, build_health
from pysoarm.exceptions import CommunicationError
from pysoarm.models import SO101


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now


def make_read(
    voltage: int = 119, temperature: int = 35, speed: int = 7, current: int = 3
) -> tuple[Callable[[str, int | None], dict[int, int]], list[tuple[str, int | None]]]:
    calls: list[tuple[str, int | None]] = []

    def read(register: str, retries: int | None = None) -> dict[int, int]:
        calls.append((register, retries))
        values = {
            "present_voltage": voltage,
            "present_temperature": temperature,
            "present_speed": speed,
            "present_current": current,
        }
        return dict.fromkeys(SO101.motor_ids, values[register])

    return read, calls


def test_cache_is_empty_before_the_first_refresh() -> None:
    clock = FakeClock()
    cache = HealthCache(SO101.motor_ids, clock)
    assert math.isinf(cache.age(clock()))
    assert cache.voltage_v == (0.0,) * SO101.joint_count
    assert cache.temperature_c == (0,) * SO101.joint_count
    assert cache.speed_raw == (0,) * SO101.joint_count
    assert cache.current == (0,) * SO101.joint_count


def test_refresh_all_converts_units_and_marks_fresh() -> None:
    clock = FakeClock()
    cache = HealthCache(SO101.motor_ids, clock)
    read, calls = make_read()
    cache.refresh_all(read, retries=2)
    assert [name for name, _ in calls] == list(HEALTH_REGISTERS)
    assert all(retries == 2 for _, retries in calls)
    assert cache.voltage_v == (11.9,) * SO101.joint_count
    assert cache.temperature_c == (35,) * SO101.joint_count
    assert cache.speed_raw == (7,) * SO101.joint_count
    assert cache.current == (3,) * SO101.joint_count
    assert cache.age(clock()) == 0.0


def test_incremental_refresh_cycles_and_ages() -> None:
    clock = FakeClock()
    cache = HealthCache(SO101.motor_ids, clock)
    read, calls = make_read()
    cache.refresh_incremental(read)
    assert [name for name, _ in calls] == ["present_speed"]
    assert math.isinf(cache.age(clock()))  # not a complete refresh yet
    clock.now += 0.1
    for _ in range(len(HEALTH_REGISTERS) - 1):
        cache.refresh_incremental(read)
    assert [name for name, _ in calls] == list(HEALTH_REGISTERS)
    assert cache.age(clock()) == 0.0
    clock.now += 0.25
    assert cache.age(clock()) == pytest.approx(0.25)


def test_latched_status_flags_are_per_motor() -> None:
    cache = HealthCache(SO101.motor_ids, FakeClock())
    cache.latch(
        CommunicationError(
            "alarm",
            motor_id=3,
            status=0x08,
            transient=True,
        )
    )
    assert cache.flags_for(3) == frozenset({"overcurrent"})
    assert cache.flags_for(1) == frozenset()
    cache.latch(CommunicationError("no status", motor_id=4))
    assert cache.flags_for(4) == frozenset()


def test_build_health_combines_latched_and_derived_flags() -> None:
    clock = FakeClock()
    cache = HealthCache(SO101.motor_ids, clock)
    read, _ = make_read(voltage=90, temperature=60)
    cache.refresh_all(read)
    cache.latch(CommunicationError("alarm", motor_id=3, status=0x08))
    health = build_health(SO101, cache, temp_warn_c=55.0, voltage_min_v=10.0)
    assert list(health) == list(SO101.joints)
    assert health["shoulder_pan"].flags == frozenset({"voltage", "overheat"})
    assert health["elbow_flex"].flags == frozenset({"voltage", "overheat", "overcurrent"})
    assert health["gripper"].voltage_v == pytest.approx(9.0)
    assert health["gripper"].current_raw == 3
    assert health["gripper"].as_dict()["motor_id"] == 6


def test_build_health_handles_partial_cache() -> None:
    cache = HealthCache(SO101.motor_ids, FakeClock())
    health = build_health(SO101, cache, temp_warn_c=55.0, voltage_min_v=10.0)
    assert health["shoulder_pan"].voltage_v == 0.0
    assert health["shoulder_pan"].temperature_c == 0
