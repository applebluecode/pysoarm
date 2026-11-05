"""Safety guards applied to every goal before it reaches the bus.

Guards are pure functions of the requested goal, the current state and the limits, so all the
rules are unit-testable without hardware. They run in this order:

1. **validity** — correct joint count, finite values, known joints (``ConfigurationError``);
2. **watchdog** — too many consecutive read failures (``WatchdogTimeout``);
3. **thermal / supply** — too hot or undervoltage (``SafetyViolation``);
4. **soft limits** — clamp to the calibrated travel minus a margin;
5. **per-step clipping** — ``|q_goal - q_now| <= step``, where ``step`` is the strictest of
   the absolute ``max_step_deg`` cap and the speed ceiling divided by the loop rate. That is
   the guard that makes the first goal after connect (and every goal after a lost frame)
   unable to cause a jump;
6. **deadband** — optional ``min_step_deg``: a command smaller than the servo deadband is
   not sent (the measured position is kept) and is reported in ``reasons``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

from pysoarm.calibration import Calibration
from pysoarm.constants import (
    DEFAULT_LOOP_HZ,
    DEFAULT_MIN_STEP_DEG,
    VOLTAGE_MIN_FOLLOWER_V,
    VOLTAGE_MIN_LEADER_V,
)
from pysoarm.exceptions import ConfigurationError, SafetyViolation, WatchdogTimeout
from pysoarm.models.base import RobotModel
from pysoarm.state import ArmState

if TYPE_CHECKING:
    from pysoarm.arm import Role

__all__ = ["SafetyLimits", "SafetyReport", "apply_guards"]


@dataclass(frozen=True, slots=True)
class SafetyLimits:
    """Thresholds for the guards.

    The default configuration puts **no software cap on the per-tick step or joint speed**,
    matching LeRobot's SO follower (``max_relative_target=None``): each goal is sent as
    requested, bounded only by the calibrated soft limits, the thermal/voltage guards and the
    servo itself. This keeps leader-follower following as direct as LeRobot's.

    A speed ceiling is available when you want one. Set :attr:`max_speed_deg_s` for a
    rate-independent limit (``720`` reproduces the value used by the pysoarm teleoperation
    validation: 14.4 deg/tick at 50 Hz, 3.6 deg/tick at 200 Hz), or :attr:`max_step_deg` for a
    fixed absolute per-tick cap. When both are set the stricter one wins.

    Args:
        max_step_deg: Absolute per-tick ceiling in degrees, independent of the loop rate.
            ``None`` (default) means no fixed step cap. Use it when a single goal must never
            exceed a fixed jump even if the loop rate is unknown.
        max_speed_deg_s: Joint-speed ceiling in degrees per second. The per-tick limit is
            derived from the loop frequency with :meth:`step_for`, so the same physical
            speed is enforced at 50 Hz and at 200 Hz. ``None`` (default) means no speed
            cap. ``720`` is the recommended safety preset for teleoperation.
        min_step_deg: Commands smaller than this are not sent: the joint keeps its measured
            position and the skip is reported in :attr:`SafetyReport.reasons`. Use it to
            avoid sub-degree ``write_goal`` calls that the servo deadband swallows. Default
            ``0.0`` keeps the historical behaviour.
        limit_margin_deg: Margin kept inside the calibrated travel, in degrees.
        temp_warn_c: Temperature above which goals are derated.
        temp_max_c: Temperature at which goals are refused.
        voltage_min_v: Supply voltage below which goals are refused. The default matches a
            12 V follower bus; :meth:`for_role` picks ``4.0 V`` for a 5 V leader.
        max_consecutive_failures: Read failures tolerated before the watchdog trips.

    Raises:
        ConfigurationError: If the thresholds are not ordered sensibly.
    """

    max_step_deg: float | None = None
    max_speed_deg_s: float | None = None
    min_step_deg: float = DEFAULT_MIN_STEP_DEG
    limit_margin_deg: float = 1.0
    temp_warn_c: float = 55.0
    temp_max_c: float = 70.0
    voltage_min_v: float = VOLTAGE_MIN_FOLLOWER_V
    max_consecutive_failures: int = 3

    def __post_init__(self) -> None:
        if self.max_step_deg is not None and self.max_step_deg <= 0:
            raise ConfigurationError("max_step_deg must be positive")
        if self.max_speed_deg_s is not None and self.max_speed_deg_s <= 0:
            raise ConfigurationError("max_speed_deg_s must be positive")
        if self.min_step_deg < 0:
            raise ConfigurationError("min_step_deg must be non-negative")
        if self.limit_margin_deg < 0:
            raise ConfigurationError("limit_margin_deg must be non-negative")
        if not self.temp_warn_c < self.temp_max_c:
            raise ConfigurationError("temp_warn_c must be below temp_max_c")
        if self.voltage_min_v <= 0:
            raise ConfigurationError("voltage_min_v must be positive")
        if self.max_consecutive_failures < 1:
            raise ConfigurationError("max_consecutive_failures must be at least 1")

    @classmethod
    def for_role(cls, role: Role, **overrides: object) -> SafetyLimits:
        """Default limits suitable for an arm used as ``follower`` or ``leader``.

        A leader is powered from a 5 V rail and is never commanded, so it needs the lower
        voltage threshold (:data:`~pysoarm.constants.VOLTAGE_MIN_LEADER_V`) instead of the
        12 V follower threshold. Neither role gets a software speed cap by default.

        Args:
            role: ``"follower"`` or ``"leader"``.
            **overrides: Fields forwarded to the constructor, so callers can tune one value
                without restating the role defaults.

        Returns:
            A :class:`SafetyLimits` with the role's voltage threshold.

        Raises:
            ConfigurationError: On an unknown role or invalid overrides.
        """
        if role not in ("follower", "leader"):
            raise ConfigurationError(f"role must be 'follower' or 'leader', got {role!r}")
        defaults: dict[str, object] = {
            "voltage_min_v": (
                VOLTAGE_MIN_FOLLOWER_V if role == "follower" else VOLTAGE_MIN_LEADER_V
            )
        }
        defaults.update(overrides)
        return cls(**defaults)  # type: ignore[arg-type]

    def step_for(self, hz: float) -> float:
        """Per-tick step limit in degrees for a loop running at ``hz``.

        Args:
            hz: Control frequency in hertz.

        Returns:
            The strictest of ``max_step_deg`` (when set) and ``max_speed_deg_s / hz`` (when
            set). ``math.inf`` when neither is configured, which means "send the goal
            unclipped" (still subject to the soft-limit and health guards).

        Raises:
            ConfigurationError: If ``hz`` is not positive.
        """
        if hz <= 0:
            raise ConfigurationError("hz must be positive")
        speed_step = math.inf if self.max_speed_deg_s is None else self.max_speed_deg_s / hz
        absolute = math.inf if self.max_step_deg is None else self.max_step_deg
        return min(absolute, speed_step)

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
            speed ceiling is honoured at the loop's actual rate. When omitted,
            ``limits.step_for(DEFAULT_LOOP_HZ)`` is used; with the default limits that is
            ``math.inf``, so the goal is only soft-limit clamped.

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
    base_step = limits.step_for(DEFAULT_LOOP_HZ) if step_deg is None else step_deg
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
        delta = target - measured
        if limits.min_step_deg > 0 and 0 < abs(delta) < limits.min_step_deg:
            # Inside the servo's positioning deadband: sending this would only add noise.
            target = measured
            clipped = True
            reasons.append(
                f"skipped {joint} ({abs(delta):.3f} deg < min_step {limits.min_step_deg:.3f})"
            )
        out.append(target)

    if derate < 1.0:
        reasons.append(f"derated to {derate:.2f} because of motor temperature")
    return SafetyReport(goal_deg=tuple(out), clipped=clipped, derate=derate, reasons=tuple(reasons))
