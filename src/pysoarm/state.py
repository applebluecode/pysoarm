"""Immutable snapshots of the arm.

A state is produced by one :meth:`SOArm.read_once` and never touches the hardware again, so it
is safe to log, buffer and hand to a policy.
"""

from __future__ import annotations

from dataclasses import dataclass

from pysoarm.constants import SPEED_LSB_DEG_PER_S

__all__ = ["ArmState", "MotorHealth"]

#: Seconds of clock difference after which a state is considered stale.
DEFAULT_STALE_AFTER_S = 0.25


@dataclass(frozen=True, slots=True)
class ArmState:
    """One consistent snapshot of the arm.

    Every tuple is in the model's joint order (see :class:`~pysoarm.models.base.RobotModel`).

    Args:
        q_deg: Measured joint positions in degrees.
        speed_raw: Raw ``present_speed`` values (servo counts).
        current: Raw ``present_current`` values (servo counts, **not** amperes).
        temperature_c: Motor temperatures in degrees Celsius.
        voltage_v: Supply voltage per motor in volts.
        t: Monotonic timestamp of the read, in seconds.
        consecutive_failures: Failed reads since the last success (watchdog input).
        health_age_s: Age of the cached health registers at ``t``, in seconds. ``0.0`` means
            they were refreshed by this very read; a large value means ``health_period_s``
            has not elapsed or health refresh is disabled.
    """

    q_deg: tuple[float, ...]
    speed_raw: tuple[int, ...]
    current: tuple[int, ...]
    temperature_c: tuple[int, ...]
    voltage_v: tuple[float, ...]
    t: float
    consecutive_failures: int = 0
    health_age_s: float = 0.0

    @property
    def dq_deg_s(self) -> tuple[float, ...]:
        """Indicative joint velocities in deg/s.

        Derived from ``present_speed`` with the datasheet scale (0.732 rpm per LSB). This is
        **not** validated on hardware; use it for monitoring, not for control.
        """
        return tuple(value * SPEED_LSB_DEG_PER_S for value in self.speed_raw)

    @property
    def max_temperature_c(self) -> float:
        """Hottest motor in this snapshot (``-inf`` for an empty state)."""
        return max(self.temperature_c, default=float("-inf"))

    @property
    def min_voltage_v(self) -> float:
        """Lowest supply voltage in this snapshot (``inf`` for an empty state)."""
        return min(self.voltage_v, default=float("inf"))

    def is_stale(self, now: float, max_age_s: float = DEFAULT_STALE_AFTER_S) -> bool:
        """Whether the snapshot is older than ``max_age_s`` at monotonic time ``now``."""
        return now - self.t > max_age_s

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view, handy for logging and the CLI."""
        return {
            "t": self.t,
            "q_deg": list(self.q_deg),
            "dq_deg_s": list(self.dq_deg_s),
            "speed_raw": list(self.speed_raw),
            "current": list(self.current),
            "temperature_c": list(self.temperature_c),
            "voltage_v": list(self.voltage_v),
            "consecutive_failures": self.consecutive_failures,
            "health_age_s": self.health_age_s,
        }


@dataclass(frozen=True, slots=True)
class MotorHealth:
    """Structured per-motor health, pylibfranka-``Errors`` style.

    Args:
        joint: Joint name, in the model's control order.
        motor_id: Servo id on the bus.
        voltage_v: Supply voltage reported by the servo, in volts.
        temperature_c: Motor temperature, in degrees Celsius.
        current_raw: Raw ``present_current`` value (servo counts, not amperes).
        flags: Named alarm bits currently attributable to this motor. Bits observed in a
            status packet are latched; ``"voltage"`` and ``"overheat"`` are also derived
            from the measured supply and temperature so the view is useful even when no
            status packet has failed yet.
    """

    joint: str
    motor_id: int
    voltage_v: float
    temperature_c: int
    current_raw: int
    flags: frozenset[str]

    def as_dict(self) -> dict[str, object]:
        """JSON-serialisable view."""
        return {
            "joint": self.joint,
            "motor_id": self.motor_id,
            "voltage_v": self.voltage_v,
            "temperature_c": self.temperature_c,
            "current_raw": self.current_raw,
            "flags": sorted(self.flags),
        }
