"""The ``Bus`` protocol: the only boundary the control code is allowed to cross.

Above this boundary nothing imports ``serial`` or ``rustypot`` and everything is expressed in
raw register values; below it nothing knows about degrees, calibration or limits. That split
is what lets the whole control path run against :class:`~pysoarm.bus.fake.FakeBus` in tests.

Implementations must honour these rules:

* **one batch call per control step** — call ``sync_read``/``sync_write`` instead of looping
  in application code. On this hardware ``sync_read`` is internally sequential (the TTL bus is
  half-duplex), but keeping it a single call is what makes swapping in a faster backend or a
  fake painless;
* **retries are idempotent** — a retry must not change the value written;
* **raise, do not block** — a missing reply becomes
  :class:`~pysoarm.exceptions.CommunicationError` after the configured retries, with the raw
  bytes attached for post-mortem;
* **stateless** — no calibration, no unit conversion, no clipping. Those live above.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Protocol, runtime_checkable

__all__ = ["Bus"]


@runtime_checkable
class Bus(Protocol):
    """Raw register access to a chain of STS3215 servos.

    All values are raw register integers (ticks, LSBs); conversions happen above this layer.
    """

    @property
    def backend(self) -> str:
        """Short name of this transport (``"scs"``, ``"rustypot"``, ``"fake"``)."""
        ...

    @property
    def is_open(self) -> bool:
        """Whether the transport is open and usable."""
        ...

    def __enter__(self) -> Bus:
        """Open the transport and return it (``with make_bus(...) as bus:``)."""
        ...

    def __exit__(self, *exc_info: object) -> None:
        """Close the transport; must be equivalent to :meth:`close`."""
        ...

    def open(self) -> None:
        """Open the transport.

        Raises:
            BusConnectionError: If the port cannot be opened or is already in use.
        """
        ...

    def close(self) -> None:
        """Close the transport. Must be idempotent and must not raise."""
        ...

    def read(self, motor_id: int, register: str, *, retries: int | None = None) -> int:
        """Read one register of one motor.

        Args:
            motor_id: Servo id (1-253).
            register: Register name, e.g. ``"present_position"``.
            retries: Override the bus' default retry count for this call. Hot paths can
                trade one long timeout for several short attempts.

        Returns:
            The decoded integer value.

        Raises:
            ConfigurationError: Unknown register.
            CommunicationError: No valid reply after the configured retries.
        """
        ...

    def write(
        self, motor_id: int, register: str, value: int, *, retries: int | None = None
    ) -> None:
        """Write one register of one motor and wait for its acknowledgement.

        Args:
            motor_id: Servo id.
            register: Register name.
            value: Value to write.
            retries: Override the bus' default retry count for this call.

        Raises:
            ConfigurationError: Unknown or read-only register.
            CommunicationError: No valid acknowledgement after the configured retries.
        """
        ...

    def sync_read(
        self, motor_ids: Sequence[int], register: str, *, retries: int | None = None
    ) -> dict[int, int]:
        """Read one register from several motors in a single batch.

        Args:
            motor_ids: Servo ids to read.
            register: Register name.
            retries: Override the bus' default retry count for this call.

        Returns:
            Mapping ``id -> value`` for every requested id.

        Raises:
            CommunicationError: If any id did not answer after the configured retries.
        """
        ...

    def sync_write(
        self,
        register: str,
        values: Mapping[int, int],
        *,
        retries: int | None = None,
    ) -> None:
        """Write one register of several motors in a single broadcast frame.

        The frame is fire-and-forget: there is no acknowledgement, which is what makes this
        the fast path (~0.26 ms for six goals).

        Args:
            register: Register name.
            values: Mapping ``id -> value``.
            retries: Override the bus' default retry count for this call.

        Raises:
            ConfigurationError: Unknown register.
        """
        ...

    def enable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Enable torque (``torque_enable = 1``) for the given motors, or all of them."""
        ...

    def disable_torque(self, motor_ids: Sequence[int] | None = None) -> None:
        """Disable torque (``torque_enable = 0``) for the given motors, or all of them."""
        ...

    def scan(self, motor_ids: Sequence[int] | None = None) -> dict[int, int]:
        """Probe ids and report their model numbers.

        Args:
            motor_ids: Candidate ids; ``None`` probes the default 1-16 range.

        Returns:
            Mapping ``id -> model_number`` for the servos that answered.
        """
        ...
