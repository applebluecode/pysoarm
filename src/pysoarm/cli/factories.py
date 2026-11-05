"""Build the objects a command operates on from parsed arguments.

The factories are the single seam tests replace with in-memory fakes. Commands import this
module (not individual factories) so a test can redirect every command with one patch on
``pysoarm.cli.factories``.
"""

from __future__ import annotations

import argparse
import importlib
from collections.abc import Callable
from typing import cast

from pysoarm.arm import SOArm
from pysoarm.bus import make_bus as build_bus
from pysoarm.bus.base import Bus
from pysoarm.kinematics.base import Kinematics
from pysoarm.models.base import get_model

__all__ = ["make_arm", "make_bus", "make_kinematics", "make_readonly_arm"]


def make_bus(args: argparse.Namespace) -> Bus:
    """Build the raw transport for a command."""
    model = get_model(args.model)
    return build_bus(
        args.port,
        model.motor_ids,
        backend=args.backend,
        servo_model=model.servo_model,
        baudrate=args.baudrate or model.baudrate,
    )


def make_arm(args: argparse.Namespace) -> SOArm:
    """Build a writable arm for ``read``/``monitor``/``hold``."""
    return SOArm(
        args.port,
        model=args.model,
        backend=args.backend,
        calibration=args.calibration,
        baudrate=args.baudrate,
    )


def make_kinematics(args: argparse.Namespace) -> Kinematics:
    """Build the Cartesian solver for ``pose``/``ik``."""
    # lazy: the placo adapter needs numpy and the optional 'kinematics' extra
    module = importlib.import_module("pysoarm.kinematics")
    factory = cast("Callable[..., Kinematics]", module.PlacoKinematics)
    return factory(args.urdf, target_frame=args.target_frame)


def make_readonly_arm(args: argparse.Namespace) -> SOArm:
    """Arm used by ``pose --port``: no register writes, torque stays off."""
    return SOArm(
        args.port,
        model=args.model,
        backend=args.backend,
        calibration=args.calibration,
        configure_motors=False,
        enable_torque=False,
    )
