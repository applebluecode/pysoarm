"""Exception hierarchy.

Every error raised by this package derives from :class:`PysoarmError`, so callers can catch
one type without swallowing unrelated failures (``OSError`` from ``pyserial``, ``ValueError``
from user code, ...).

Physical safety is treated as part of the contract: :class:`WatchdogTimeout` and
:class:`SafetyViolation` must leave the arm in a defined state (torque off or last safe goal),
and that is stated in their docstrings.
"""

from __future__ import annotations

__all__ = [
    "BusConnectionError",
    "CalibrationError",
    "CommunicationError",
    "ConfigurationError",
    "KinematicsError",
    "NotReachableError",
    "PysoarmError",
    "SafetyViolation",
    "WatchdogTimeout",
]


class PysoarmError(Exception):
    """Base class for every error raised by pysoarm."""


class ConfigurationError(PysoarmError):
    """Invalid arguments, unknown joint/register, missing or malformed calibration."""


class BusConnectionError(PysoarmError):
    """The serial bus could not be opened or is used by another process."""


class CommunicationError(PysoarmError):
    """A motor did not answer, or answered with a corrupt/error status packet.

    The arm is left holding its last goal; the caller (or the watchdog) decides whether to
    keep going.

    Attributes:
        motor_id: Motor that failed, when known.
        register: Register name involved, when known.
        raw: Raw bytes received (or sent), for post-mortem; ``None`` if nothing arrived.
    """

    def __init__(
        self,
        message: str,
        *,
        motor_id: int | None = None,
        register: str | None = None,
        raw: bytes | None = None,
    ) -> None:
        super().__init__(message)
        self.motor_id = motor_id
        self.register = register
        self.raw = raw


class CalibrationError(PysoarmError):
    """Calibration is inconsistent or does not match the arm model.

    Raised before any motor is commanded, so the arm state is unchanged.
    """


class KinematicsError(PysoarmError):
    """Kinematics could not be set up or evaluated (missing URDF, bad joint list, ...)."""


class NotReachableError(KinematicsError):
    """Inverse kinematics did not converge to the requested pose.

    Raised *before* any goal is written, so the arm keeps its current target. The residual
    distance is included in the message.
    """


class SafetyViolation(PysoarmError):
    """A guard rejected a goal because executing it would be unsafe.

    The arm keeps its previous goal; no new goal is written.
    """


class WatchdogTimeout(SafetyViolation):
    """Bus communication was lost for longer than the watchdog allows.

    The caller must disable torque (``SOArm`` does so before re-raising) — a servo holding a
    stale goal while the host believes it is communicating is exactly the failure mode this
    guard exists for.
    """
