"""Per-arm descriptions.

A model fixes joint order, motor ids and gripper caps; everything else takes a
:class:`~pysoarm.models.base.RobotModel` as an argument.
"""

from __future__ import annotations

from pysoarm.models.base import MODELS, RobotModel, get_model
from pysoarm.models.so101 import SO100, SO101, SO_JOINTS

__all__ = ["MODELS", "SO100", "SO101", "SO_JOINTS", "RobotModel", "get_model"]
