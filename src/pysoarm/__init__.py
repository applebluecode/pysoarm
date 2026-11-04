"""pysoarm — lean, hardware-level Python API for SO-100/SO-101 robot arms.

The arm is six Feetech STS3215 serial-bus servos on one 1 Mbaud TTL bus. This package talks to
that bus directly: no datasets, no policies, no training stack.

Quick start::

    from pysoarm import SOArm

    with SOArm(port="/dev/so101_follower", calibration="follower.json") as arm:
        state = arm.read_once()
        arm.move_to([0, -30, 60, 0, 90, 40], duration=2.0)

Scope
-----
pysoarm is a **device-layer** SDK: teleoperation, dataset recording and policies are
deliberately out of scope (see ``docs/adr/0001-scope-device-layer.md``).
"""

from __future__ import annotations

from pysoarm._version import __version__
from pysoarm.arm import LoopStats, SOArm
from pysoarm.bus import Backend, make_bus
from pysoarm.bus.base import Bus
from pysoarm.bus.fake import FakeBus
from pysoarm.bus.registers import REGISTERS, Register, RegisterKind, get_register
from pysoarm.bus.rustypot import RustypotBus
from pysoarm.bus.scs import ScsBus
from pysoarm.calibration import Calibration, JointCalibration
from pysoarm.constants import DEFAULT_BAUDRATE, DEFAULT_LOOP_HZ
from pysoarm.exceptions import (
    BusConnectionError,
    CalibrationError,
    CommunicationError,
    ConfigurationError,
    PysoarmError,
    SafetyViolation,
    WatchdogTimeout,
)
from pysoarm.models import MODELS, SO100, SO101, RobotModel, get_model
from pysoarm.safety import SafetyLimits, SafetyReport, apply_guards
from pysoarm.state import ArmState
from pysoarm.trajectory import LinearRamp, MinimumJerk, ResampledTrajectory, Trajectory

__all__ = [
    "DEFAULT_BAUDRATE",
    "DEFAULT_LOOP_HZ",
    "MODELS",
    "REGISTERS",
    "SO100",
    "SO101",
    "ArmState",
    "Backend",
    "Bus",
    "BusConnectionError",
    "Calibration",
    "CalibrationError",
    "CommunicationError",
    "ConfigurationError",
    "FakeBus",
    "JointCalibration",
    "LinearRamp",
    "LoopStats",
    "MinimumJerk",
    "PysoarmError",
    "Register",
    "RegisterKind",
    "ResampledTrajectory",
    "RobotModel",
    "RustypotBus",
    "SOArm",
    "SafetyLimits",
    "SafetyReport",
    "SafetyViolation",
    "ScsBus",
    "Trajectory",
    "WatchdogTimeout",
    "__version__",
    "apply_guards",
    "get_model",
    "get_register",
    "make_bus",
]
