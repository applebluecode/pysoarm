"""Immutable snapshots of the arm.

A state is produced by one :meth:`SOArm.read_once` and never touches the hardware again, so it
is safe to log, buffer and hand to a policy.
"""

from __future__ import annotations

from dataclasses import dataclass

from pysoarm.constants import SPEED_LSB_DEG_PER_S

__all__ = ["ArmState"]

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
    """

    q_deg: tuple[float, ...]
    speed_raw: tuple[int, ...]
    current: tuple[int, ...]
    temperature_c: tuple[int, ...]
    voltage_v: tuple[float, ...]
    t: float
    consecutive_failures: int = 0

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
        }
