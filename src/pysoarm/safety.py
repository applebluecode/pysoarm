"""Safety guards applied to every goal before it reaches the bus.

Guards are pure functions of the requested goal, the current state and the limits, so all the
rules are unit-testable without hardware. They run in this order:

1. **validity** — correct joint count, finite values, known joints (``ConfigurationError``);
2. **watchdog** — too many consecutive read failures (``WatchdogTimeout``);
3. **thermal / supply** — too hot or undervoltage (``SafetyViolation``);
4. **soft limits** — clamp to the calibrated travel minus a margin;
5. **per-step clipping** — ``|q_goal - q_now| <= max_step_deg * derate``, the guard that makes
   the first goal after connect (and every goal after a lost frame) unable to cause a jump.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from pysoarm.calibration import Calibration
from pysoarm.exceptions import ConfigurationError, SafetyViolation, WatchdogTimeout
from pysoarm.models.base import RobotModel
from pysoarm.state import ArmState

__all__ = ["SafetyLimits", "SafetyReport", "apply_guards"]


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    """Thresholds for the guards.

    Args:
        max_step_deg: Largest allowed change of a joint between two goals, in degrees.
        max_speed_deg_s: Optional joint-speed ceiling in degrees per second. When set, the
            per-tick limit is derived from the loop frequency with :meth:`step_for`, so the
            same physical speed is enforced at 50 Hz and at 200 Hz. The stricter of this and
            ``max_step_deg`` wins. ``None`` keeps the historical fixed-step behaviour.
        limit_margin_deg: Margin kept inside the calibrated travel, in degrees.
        temp_warn_c: Temperature above which goals are derated.
        temp_max_c: Temperature at which goals are refused.
        voltage_min_v: Supply voltage below which goals are refused.
        max_consecutive_failures: Read failures tolerated before the watchdog trips.

    Raises:
        ConfigurationError: If the thresholds are not ordered sensibly.
    """

    max_step_deg: float = 5.0
    max_speed_deg_s: float | None = None
    limit_margin_deg: float = 1.0
    temp_warn_c: float = 55.0
    temp_max_c: float = 70.0
    voltage_min_v: float = 10.0
    max_consecutive_failures: int = 3

    def __post_init__(self) -> None:
        if self.max_step_deg <= 0:
            raise ConfigurationError("max_step_deg must be positive")
        if self.max_speed_deg_s is not None and self.max_speed_deg_s <= 0:
            raise ConfigurationError("max_speed_deg_s must be positive")
        if self.limit_margin_deg < 0:
            raise ConfigurationError("limit_margin_deg must be non-negative")
        if not self.temp_warn_c < self.temp_max_c:
            raise ConfigurationError("temp_warn_c must be below temp_max_c")
        if self.voltage_min_v <= 0:
            raise ConfigurationError("voltage_min_v must be positive")
        if self.max_consecutive_failures < 1:
            raise ConfigurationError("max_consecutive_failures must be at least 1")

    def step_for(self, hz: float) -> float:
        """Per-tick step limit in degrees for a loop running at ``hz``.

        Args:
            hz: Control frequency in hertz.

        Returns:
            ``min(max_step_deg, max_speed_deg_s / hz)`` when a speed ceiling is configured,
            otherwise ``max_step_deg``.

        Raises:
            ConfigurationError: If ``hz`` is not positive.
        """
        if hz <= 0:
            raise ConfigurationError("hz must be positive")
        if self.max_speed_deg_s is None:
            return self.max_step_deg
        return min(self.max_step_deg, self.max_speed_deg_s / hz)

    def derate(self, temperature_c: Sequence[int]) -> float:
        """Speed factor in ``(0.25, 1.0]`` from the hottest motor.

        ``1.0`` below ``temp_warn_c``, linearly down to ``0.25`` at ``temp_max_c``.
        """
        hottest = max(temperature_c, default=self.temp_warn_c)
        if hottest <= self.temp_warn_c:
            return 1.0
        span = self.temp_max_c - self.temp_warn_c
        fraction = min(1.0, (hottest - self.temp_warn_c) / span)
        return 1.0 - 0.75 * fraction


@dataclass(frozen=True, slots=True)
class SafetyReport:
    """Outcome of the guards: the goal that may be sent, and why it changed.

    Args:
        goal_deg: Goal to send (already clamped/clipped), in the model's joint order.
        clipped: Whether any value was changed by a guard.
        derate: Speed factor applied because of temperature.
        reasons: Human-readable reasons, empty when nothing changed.

    ``write_goal`` returns this object's ``goal_deg`` so clipping is observable, never silent.
    """

    goal_deg: tuple[float, ...]
    clipped: bool
    derate: float
    reasons: tuple[str, ...]


def _check_goal(goal: Sequence[float], model: RobotModel) -> tuple[float, ...]:
    if len(goal) != model.joint_count:
        raise ConfigurationError(
            f"Expected {model.joint_count} joint values for {model.name}, got {len(goal)}"
        )
    values = tuple(float(value) for value in goal)
    for joint, value in zip(model.joints, values, strict=True):
        if not math.isfinite(value):
            raise SafetyViolation(f"Joint {joint!r} received a non-finite goal ({value!r})")
    return values


def _check_health(state: ArmState, limits: SafetyLimits) -> None:
    if state.consecutive_failures >= limits.max_consecutive_failures:
        raise WatchdogTimeout(
            f"{state.consecutive_failures} consecutive read failures "
            f"(limit {limits.max_consecutive_failures}); torque must be disabled"
        )
    hottest = state.max_temperature_c
    if hottest >= limits.temp_max_c:
        raise SafetyViolation(
            f"Motor at {hottest:.0f} C is at or above the {limits.temp_max_c:.0f} C limit"
        )
    coldest_voltage = state.min_voltage_v
    if coldest_voltage < limits.voltage_min_v:
        raise SafetyViolation(
            f"Supply at {coldest_voltage:.1f} V is below the {limits.voltage_min_v:.1f} V limit"
        )


def apply_guards(
    goal_deg: Sequence[float],
    *,
    state: ArmState,
    calibration: Calibration,
    model: RobotModel,
    limits: SafetyLimits,
    step_deg: float | None = None,
) -> SafetyReport:
    """Apply every guard to ``goal_deg``.

    Args:
        goal_deg: Requested goal in degrees, in the model's joint order.
        state: Most recent state (provides the measured position and the health signals).
        calibration: Joint calibration, used for the soft limits.
        model: Arm model, for joint names.
        limits: Guard thresholds.
        step_deg: Per-tick step limit override, normally ``limits.step_for(hz)`` so that a
            speed ceiling is honoured at the loop's actual rate.

    Returns:
        A :class:`SafetyReport` with the goal that may be sent.

    Raises:
        ConfigurationError: Wrong number of joints.
        SafetyViolation: Non-finite goal, over-temperature, undervoltage.
        WatchdogTimeout: Too many consecutive read failures.
    """
    values = _check_goal(goal_deg, model)
    _check_health(state, limits)
    if len(state.q_deg) != model.joint_count:
        raise ConfigurationError(
            f"State has {len(state.q_deg)} joints, model {model.name!r} has {model.joint_count}"
        )

    derate = limits.derate(state.temperature_c)
    base_step = limits.max_step_deg if step_deg is None else step_deg
    if base_step <= 0:
        raise ConfigurationError("step_deg must be positive")
    max_step = base_step * derate
    reasons: list[str] = []
    clipped = False
    out: list[float] = []

    for joint, wanted, measured in zip(model.joints, values, state.q_deg, strict=True):
        cal = calibration.for_joint(joint)
        lo = cal.raw_to_deg(cal.range_min) + limits.limit_margin_deg
        hi = cal.raw_to_deg(cal.range_max) - limits.limit_margin_deg
        lo, hi = min(lo, hi), max(lo, hi)
        target = min(max(wanted, lo), hi)
        if target != wanted:
            clipped = True
            reasons.append(f"clamped {joint} to {target:.2f} deg (soft limit)")
        step = target - measured
        if abs(step) > max_step:
            target = measured + math.copysign(max_step, step)
            clipped = True
            reasons.append(f"clipped {joint} step to {target:.2f} deg (max {max_step:.2f})")
        out.append(target)

    if derate < 1.0:
        reasons.append(f"derated to {derate:.2f} because of motor temperature")
    return SafetyReport(goal_deg=tuple(out), clipped=clipped, derate=derate, reasons=tuple(reasons))
