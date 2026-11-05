"""Cartesian dry-run commands: ``pose`` and ``ik``."""

from __future__ import annotations

import argparse
import json

from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_OK, EXIT_USAGE
from pysoarm.exceptions import ConfigurationError
from pysoarm.kinematics.base import Pose

__all__ = ["_cmd_ik", "_cmd_pose", "_joint_list"]


def _joint_list(text: str) -> tuple[float, ...]:
    """Parse ``"0, -30 60"`` into a tuple of floats."""
    cleaned = text.replace(",", " ").split()
    try:
        return tuple(float(value) for value in cleaned)
    except ValueError as exc:
        raise ConfigurationError(f"Cannot parse joint values from {text!r}: {exc}") from exc


def _cmd_pose(args: argparse.Namespace) -> int:
    kinematics = factories.make_kinematics(args)
    if args.q is not None:
        q_deg = _joint_list(args.q)
    elif args.port is not None:
        with factories.make_readonly_arm(args) as arm:
            q_deg = arm.read_once(health=False).q_deg
    else:
        print_error("pass either --q or --port")
        return EXIT_USAGE
    pose = kinematics.forward(q_deg)
    if args.json:
        print(json.dumps({"joints_deg": list(q_deg), **pose.as_dict()}, indent=2))
    else:
        x, y, z, roll, pitch, yaw = pose.to_xyz_rpy()
        print(f"xyz_m   {x:8.4f} {y:8.4f} {z:8.4f}")
        print(f"rpy_deg {roll:8.2f} {pitch:8.2f} {yaw:8.2f}")
    return EXIT_OK


def _cmd_ik(args: argparse.Namespace) -> int:
    kinematics = factories.make_kinematics(args)
    roll, pitch, yaw = args.rpy
    target = Pose.from_xyz_rpy(*args.xyz, roll=roll, pitch=pitch, yaw=yaw)
    seed = _joint_list(args.seed) if args.seed else None
    solution = kinematics.inverse(target, seed_deg=seed)
    if args.json:
        print(json.dumps({"joints_deg": list(solution)}, indent=2))
    else:
        print(" ".join(f"{value:8.2f}" for value in solution))
    return EXIT_OK
