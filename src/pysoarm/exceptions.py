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
    "STATUS_FLAGS",
    "BusConnectionError",
    "CalibrationError",
    "CommunicationError",
    "ConfigurationError",
    "KinematicsError",
    "NotReachableError",
    "PysoarmError",
    "SafetyViolation",
    "WatchdogTimeout",
    "status_flags",
]

#: Named bits of the SCS/STS ``ERROR`` byte, in wire order. The names follow the Feetech
#: manual (bit 0 is the least significant bit).
STATUS_FLAGS: tuple[tuple[int, str], ...] = (
    (0x01, "voltage"),
    (0x02, "sensor"),
    (0x04, "overheat"),
    (0x08, "overcurrent"),
    (0x10, "angle"),
    (0x20, "overload"),
)


def status_flags(status: int) -> frozenset[str]:
    """Decode an SCS/STS status byte into the set of named alarm bits.

    Args:
        status: The ``ERROR`` byte from a status packet (``0`` means healthy).

    Returns:
        The names of the set bits, e.g. ``frozenset({"overcurrent"})``.
    """
    return frozenset(name for bit, name in STATUS_FLAGS if status & bit)


class PysoarmError(Exception):
    """Base class for every error raised by pysoarm."""


class ConfigurationError(PysoarmError):
    """Invalid arguments, unknown joint/register, missing or malformed calibration."""


class BusConnectionError(PysoarmError):
    """The serial bus could not be opened or is used by another process."""


class CommunicationError(PysoarmError):
    """A motor did not answer, or answered with a corrupt/error status packet.

    The arm is left holding its last goal; the caller (or the watchdog) decides whether to
    keep going.  The structured fields make that decision mechanical instead of textual::

        try:
            bus.sync_read(ids, "present_position")
        except CommunicationError as exc:
            if exc.transient and exc.attempts < 5:
                ...  # retry the whole transaction

    Attributes:
        motor_id: Motor that failed, when known.
        register: Register name involved, when known.
        raw: Raw bytes received (or sent), for post-mortem; ``None`` if nothing arrived.
        status: The SCS/STS ``ERROR`` byte, or ``None`` when the failure was not a status
            packet (timeout, checksum, wrong id, short payload).
        transient: Whether repeating the identical, idempotent transaction may succeed.
            No-reply, checksum, truncated frames and servo alarm bits are transient; a
            mis-addressed or malformed request is not (that is a ``ConfigurationError``).
        attempts: Number of attempts actually made, including the failed one.

    The :attr:`flags` property decodes :attr:`status` into the named alarm bits from
    :data:`STATUS_FLAGS`.
    """

    def __init__(
        self,
        message: str,
        *,
        motor_id: int | None = None,
        register: str | None = None,
        raw: bytes | None = None,
        status: int | None = None,
        transient: bool = True,
        attempts: int = 1,
    ) -> None:
        super().__init__(message)
        self.motor_id = motor_id
        self.register = register
        self.raw = raw
        self.status = status
        self.transient = transient
        self.attempts = attempts

    @property
    def flags(self) -> frozenset[str]:
        """Named alarm bits carried by :attr:`status` (empty for non-status failures)."""
        return frozenset() if self.status is None else status_flags(self.status)


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
