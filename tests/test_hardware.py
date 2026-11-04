"""Opt-in smoke tests against a real arm.

Run with::

    PYSOARM_HARDWARE_PORT=/dev/ttyACM0 uv run pytest -m hardware

The tests are read-mostly: they scan the bus, calibrate from the motors, read a state, and
measure the control-step latency. Torque is enabled only for the duration of ``SOArm`` and
released in ``stop()``; no motion is commanded.
"""

from __future__ import annotations

import statistics
import time

import pytest

from pysoarm.arm import SOArm
from pysoarm.bus import make_bus
from pysoarm.bus.scs import ScsBus
from pysoarm.models import SO101

pytestmark = pytest.mark.hardware

#: A read+write control step must fit a 200 Hz period (5 ms); measured mean is 2.7 ms.
MAX_STEP_S = 0.004


def test_scan_finds_every_motor(hardware_port: str) -> None:
    with ScsBus(hardware_port, SO101.motor_ids) as bus:
        found = bus.scan()
    assert sorted(found) == list(SO101.motor_ids), f"only found {found}"


def test_state_is_plausible(hardware_port: str) -> None:
    with SOArm(hardware_port) as arm:
        state = arm.read_once(health=True)
    assert len(state.q_deg) == SO101.joint_count
    assert all(-180.0 <= value <= 180.0 for value in state.q_deg)
    assert all(9.0 <= volts <= 13.0 for volts in state.voltage_v)
    assert all(0.0 < temp < 70.0 for temp in state.temperature_c)


def test_control_step_stays_within_budget(hardware_port: str) -> None:
    with SOArm(hardware_port) as arm:
        state = arm.read_once(health=True)
        samples: list[float] = []
        for _ in range(50):
            started = time.perf_counter()
            arm.write_goal(state.q_deg)  # goal == measured pose: nothing moves
            arm.read_once(health=False)
            samples.append(time.perf_counter() - started)
    mean = statistics.mean(samples)
    worst = max(samples)
    assert mean < MAX_STEP_S, f"mean control step {mean * 1e3:.2f} ms exceeds budget"
    assert worst < MAX_STEP_S * 3, f"worst control step {worst * 1e3:.2f} ms is too high"


def test_backends_agree_on_positions(hardware_port: str) -> None:
    pytest.importorskip("rustypot", reason='needs the extra: pip install "pysoarm[rustypot]"')

    with make_bus(hardware_port, SO101.motor_ids, backend="scs") as bus:
        scs = bus.sync_read(SO101.motor_ids, "present_position")
    with make_bus(hardware_port, SO101.motor_ids, backend="rustypot") as bus:
        rustypot_values = bus.sync_read(SO101.motor_ids, "present_position")
    for mid in SO101.motor_ids:
        # the arm is limp between the two reads, so allow a few ticks of drift (~0.4 deg)
        assert abs(scs[mid] - rustypot_values[mid]) <= 5, (mid, scs[mid], rustypot_values[mid])


def test_rustypot_backend_stays_within_budget(hardware_port: str) -> None:
    pytest.importorskip("rustypot", reason='needs the extra: pip install "pysoarm[rustypot]"')
    with SOArm(hardware_port, backend="rustypot") as arm:
        state = arm.read_once(health=True)
        samples: list[float] = []
        for _ in range(50):
            started = time.perf_counter()
            arm.write_goal(state.q_deg)
            arm.read_once(health=False)
            samples.append(time.perf_counter() - started)
    mean = statistics.mean(samples)
    assert mean < 0.002, f"rustypot control step {mean * 1e3:.2f} ms exceeds 2 ms"
