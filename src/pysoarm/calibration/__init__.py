"""Calibration: data model and the motor-side calibration flow.

The public names are unchanged from the former single-module layout:

.. code-block:: python

    from pysoarm.calibration import Calibration, calibrate, read_calibration

:mod:`pysoarm.calibration.model` owns the data and conversions;
:mod:`pysoarm.calibration.flow` owns the bus procedure.
"""

from __future__ import annotations

from pysoarm.calibration.flow import (
    FULL_TURN_JOINTS,
    calibrate,
    read_calibration,
    record_ranges,
    reset_calibration,
    set_half_turn_homings,
    write_calibration,
)
from pysoarm.calibration.model import Calibration, JointCalibration

__all__ = [
    "FULL_TURN_JOINTS",
    "Calibration",
    "JointCalibration",
    "calibrate",
    "read_calibration",
    "record_ranges",
    "reset_calibration",
    "set_half_turn_homings",
    "write_calibration",
]
