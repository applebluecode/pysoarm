"""CLI exit codes and diagnostic thresholds shared by command modules."""

from __future__ import annotations

__all__ = ["EXIT_ERROR", "EXIT_OK", "EXIT_USAGE", "HOT_MOTOR_C"]

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_USAGE = 2
#: Temperature above which ``doctor --stress`` warns, in degrees Celsius.
HOT_MOTOR_C = 60.0
