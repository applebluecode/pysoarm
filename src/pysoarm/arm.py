"""``SOArm`` — the high-level device object.

Shaped after ``pylibfranka.Robot`` / ``pyAgxArm`` so calling code reads the same way, within
what position-servo hardware allows::

    with SOArm(port="/dev/so101_follower", calibration="follower.json") as arm:
        state = arm.read_once()
        arm.move_to([0, -30, 60, 0, 90, 40], duration=2.0)
        arm.run_control_loop(lambda s, t: policy(s), hz=200, duration=5.0)

Design rules:

* **hold before you move** — ``connect()`` reads the measured pose and writes it back as the
  first goal, so the first command can never be a jump;
* **clipping is observable** — ``write_goal()`` returns the goal actually sent, never silently
  reduced;
* **one sync read + one sync write per control tick** — health registers are cached and
  refreshed every ``health_period_s`` instead of on every tick;
* **``stop()`` is idempotent and safe from ``finally``/signal handlers** — it disables torque
  then closes the bus, in that order.
"""

from __future__ import annotations

import contextlib
import time
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Literal

from pysoarm.bus import Backend, make_bus
from pysoarm.bus.base import Bus
from pysoarm.calibration import Calibration, read_calibration, write_calibration
from pysoarm.constants import (
    DEFAULT_ACCELERATION,
    DEFAULT_D_COEFFICIENT,
    DEFAULT_HEALTH_PERIOD_S,
    DEFAULT_I_COEFFICIENT,
    DEFAULT_LOOP_HZ,
    DEFAULT_MAXIMUM_ACCELERATION,
    DEFAULT_P_COEFFICIENT,
    DEFAULT_RETRIES,
    DEFAULT_RETURN_DELAY_TIME,
    DEFAULT_TIMEOUT_S,
    OPERATING_MODE_POSITION,
    PHASE_KEEP_SINGLE_TURN,
)
from pysoarm.diagnostics import HealthCache, build_health
from pysoarm.exceptions import (
    BusConnectionError,
    CalibrationError,
    CommunicationError,
    ConfigurationError,
    WatchdogTimeout,
)
from pysoarm.gripper import Gripper
from pysoarm.kinematics.base import Kinematics, Pose
from pysoarm.loop import ErrorPolicy, LoopStats, run_loop, validate_loop_options
from pysoarm.models.base import RobotModel, get_model
from pysoarm.safety import SafetyLimits, SafetyReport, apply_guards
from pysoarm.state import ArmState, MotorHealth
from pysoarm.trajectory import LinearRamp, MinimumJerk, Trajectory
from pysoarm.tuning import read_motor_config, write_motor_config

__all__ = ["ErrorPolicy", "LoopStats", "Role", "SOArm"]

#: How the arm is used: a ``follower`` accepts goals and torque; a ``leader`` is read-only.
Role = Literal["follower", "leader"]

#: Extra settle time after a blocking move, so the reported state reflects the endpoint.
_SETTLE_S = 0.02


class SOArm:
    """One SO-100/SO-101 arm on a serial bus.

    Args:
        port: Serial device, e.g. ``"/dev/so101_follower"``. Required unless ``bus`` is given.
        bus: Pre-built transport (tests inject :class:`~pysoarm.bus.fake.FakeBus`).
        backend: Transport to build when ``bus`` is not given: ``"auto"`` (default:
            rustypot when installed, else scs), ``"scs"`` or ``"rustypot"`` (optional
            extra, ~2x faster reads).
        role: ``"follower"`` (default) accepts goals and enables torque; ``"leader"`` is a
            read-only arm: no goal is written, torque stays off, and only the registers that
            affect readability (return delay, phase, operating mode) are configured.
        model: Model name (``"so101"``) or a :class:`~pysoarm.models.base.RobotModel`.
        calibration: :class:`~pysoarm.calibration.Calibration`, a LeRobot JSON path, or
            ``None`` to read the calibration stored in the motors.
        limits: Safety thresholds.
        baudrate: Override the model's baudrate.
        timeout_s: Per-transaction timeout.
        retries: Extra attempts per transaction.
        configure_motors: Write PID/acceleration/mode/limits at connect (recommended).
        enable_torque: Enable torque at connect; with ``False`` the arm stays limp and
            ``write_goal`` is a no-op on real hardware.
        kinematics: Optional forward/inverse kinematics; required only for
            :meth:`forward` and :meth:`move_to_pose`.
        health_period_s: How often to refresh temperature/voltage/current; ``0`` refreshes
            on every read, a large value keeps the hot path at one sync read.
        clock: Monotonic clock, injectable for tests.
        sleep: Sleep function, injectable for tests.

    Raises:
        ConfigurationError: If neither ``port`` nor ``bus`` is given, or arguments conflict.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        bus: Bus | None = None,
        backend: Backend = "auto",
        role: Role = "follower",
        model: str | RobotModel = "so101",
        calibration: Calibration | str | Path | None = None,
        limits: SafetyLimits | None = None,
        baudrate: int | None = None,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        retries: int = DEFAULT_RETRIES,
        configure_motors: bool = True,
        enable_torque: bool = True,
        kinematics: Kinematics | None = None,
        health_period_s: float = DEFAULT_HEALTH_PERIOD_S,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if bus is None and port is None:
            raise ConfigurationError("pass either port= or bus=")
        if health_period_s < 0:
            raise ConfigurationError("health_period_s must be non-negative")
        if role not in ("follower", "leader"):
            raise ConfigurationError(f"role must be 'follower' or 'leader', got {role!r}")
        self._model = get_model(model) if isinstance(model, str) else model
        self._limits = limits if limits is not None else SafetyLimits()
        self._role: Role = role
        if bus is not None:
            self._bus = bus
        else:
            assert port is not None  # narrowed by the check above
            self._bus = make_bus(
                port,
                self._model.motor_ids,
                backend=backend,
                servo_model=self._model.servo_model,
                baudrate=baudrate or self._model.baudrate,
                timeout_s=timeout_s,
                retries=retries,
            )
        self._backend: str = self._bus.backend
        self._calibration = self._load_calibration(calibration)
        self._kinematics = kinematics
        self._configure_motors = configure_motors
        self._enable_torque = enable_torque
        self._health_period_s = health_period_s
        self._clock = clock
        self._sleep = sleep
        self._connected = False
        self._state: ArmState | None = None
        self._failures = 0
        self._health = HealthCache(self._model.motor_ids, clock)
        self._speed_scale = 1.0
        self._last_hz = DEFAULT_LOOP_HZ
        self._gripper: Gripper | None = None

    # -- properties -------------------------------------------------------- #
    @property
    def backend(self) -> str:
        """Transport actually in use (``"scs"``, ``"rustypot"`` or ``"fake"``)."""
        return self._backend

    @property
    def role(self) -> Role:
        """``"follower"`` or ``"leader"``."""
        return self._role

    @property
    def is_read_only(self) -> bool:
        """Whether this arm refuses goals (leader mode)."""
        return self._role == "leader"

    @property
    def speed_scale(self) -> float:
        """Multiplier applied to every per-tick step limit (default ``1.0``)."""
        return self._speed_scale

    @property
    def kinematics(self) -> Kinematics | None:
        """Cartesian solver attached to this arm, if any."""
        return self._kinematics

    @property
    def model(self) -> RobotModel:
        """Static description of this arm."""
        return self._model

    @property
    def calibration(self) -> Calibration | None:
        """Current calibration (available after ``connect()`` when read from the motors)."""
        return self._calibration

    @property
    def limits(self) -> SafetyLimits:
        """Active safety thresholds."""
        return self._limits

    @property
    def is_connected(self) -> bool:
        """Whether ``connect()`` succeeded and ``stop()`` has not run."""
        return self._connected

    @property
    def state(self) -> ArmState | None:
        """Most recent state, or ``None`` before the first read."""
        return self._state

    # -- construction helpers ---------------------------------------------- #
    @staticmethod
    def _load_calibration(
        calibration: Calibration | str | Path | None,
    ) -> Calibration | None:
        if calibration is None or isinstance(calibration, Calibration):
            return calibration
        return Calibration.from_json(calibration)

    # -- lifecycle --------------------------------------------------------- #
    def connect(self, *, allow_model_mismatch: bool = False) -> None:
        """Open the bus, configure the motors, hold the current pose, enable torque.

        The sequence is deliberately conservative: torque is off while the motors are
        configured, the measured pose is written back as the first goal, and only then is
        torque enabled. A leader is never commanded and its torque is never enabled.

        Args:
            allow_model_mismatch: Accept servos whose ``model_number`` differs from the
                model's expectation (useful for clones or a mixed bench).

        Raises:
            BusConnectionError: Port unavailable or a motor did not answer.
            ConfigurationError: The servos report an unexpected model number.
            CalibrationError: Calibration missing, incomplete or mismatched.
            CommunicationError: A register transaction failed after retries.
        """
        if self._connected:
            raise ConfigurationError("arm is already connected")
        self._bus.open()
        try:
            found = self._bus.scan(self._model.motor_ids)
            missing = [mid for mid in self._model.motor_ids if mid not in found]
            if missing:
                raise BusConnectionError(
                    f"Motors {missing} did not answer on the bus; got {sorted(found)}. "
                    "Check power, wiring and the motor ids."
                )
            expected = self._model.servo_model_number
            if expected is not None and not allow_model_mismatch:
                wrong = {
                    mid: number
                    for mid, number in found.items()
                    if mid in self._model.motor_ids and number != expected
                }
                if wrong:
                    raise ConfigurationError(
                        f"Unexpected servo model number(s) {wrong}; model "
                        f"{self._model.name!r} expects {expected} "
                        f"({self._model.servo_model}). Pass allow_model_mismatch=True to "
                        "override."
                    )
            if self._calibration is None:
                self._calibration = self._read_calibration_from_motors()
            self._calibration.validate(self._model)
            self._bus.disable_torque(self._model.motor_ids)
            if self._configure_motors:
                self._configure()
            state = self.read_once(health=True)
            if self._role == "follower":
                self.write_goal(state.q_deg)
                if self._enable_torque:
                    self._bus.enable_torque(self._model.motor_ids)
            self._state = self.read_once(health=False)
            self._connected = True
        except BaseException:
            with contextlib.suppress(Exception):
                self._bus.close()
            raise

    def stop(self) -> None:
        """Disable torque (best effort) and close the bus. Idempotent; never raises."""
        if self._bus.is_open:
            with contextlib.suppress(Exception):
                self._bus.disable_torque(self._model.motor_ids)
            with contextlib.suppress(Exception):
                self._bus.close()
        self._connected = False

    def __enter__(self) -> SOArm:
        self.connect()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    def __repr__(self) -> str:
        return (
            f"SOArm(model={self._model.name!r}, joints={self._model.joint_count}, "
            f"connected={self._connected})"
        )

    # -- configuration ----------------------------------------------------- #
    def _configure(self) -> None:
        ids = self._model.motor_ids
        constants: tuple[tuple[str, int], ...] = (
            ("return_delay_time", DEFAULT_RETURN_DELAY_TIME),
            ("acceleration", DEFAULT_ACCELERATION),
            ("maximum_acceleration", DEFAULT_MAXIMUM_ACCELERATION),
            ("operating_mode", OPERATING_MODE_POSITION),
            ("p_coefficient", DEFAULT_P_COEFFICIENT),
            ("i_coefficient", DEFAULT_I_COEFFICIENT),
            ("d_coefficient", DEFAULT_D_COEFFICIENT),
        )
        if self._role == "leader":
            # A leader must stay limp and readable: writing limits or PID gains would
            # change its feel and could even latch a fault. Only frame timing and mode.
            constants = tuple(
                item for item in constants if item[0] in ("return_delay_time", "operating_mode")
            )
        for register, value in constants:
            self._bus.sync_write(register, dict.fromkeys(ids, value))

        if self._role == "follower" and self._calibration is not None:
            raw = self._calibration.to_dict()
            self._bus.sync_write(
                "min_position_limit",
                {mid: raw[self._model.joint_of(mid)]["range_min"] for mid in ids},
            )
            self._bus.sync_write(
                "max_position_limit",
                {mid: raw[self._model.joint_of(mid)]["range_max"] for mid in ids},
            )

        phase = self._bus.sync_read(ids, "phase")
        self._bus.sync_write(
            "phase", {mid: value & ~PHASE_KEEP_SINGLE_TURN for mid, value in phase.items()}
        )

        gripper_id = self._model.gripper_id
        if self._role == "follower" and gripper_id is not None:
            for register, value in self._model.gripper_register_defaults:
                self._bus.write(gripper_id, register, value)

    def _read_calibration_from_motors(self) -> Calibration:
        return read_calibration(self._bus, self._model)

    # -- reading ----------------------------------------------------------- #
    def _require_calibration(self) -> Calibration:
        if self._calibration is None:
            raise CalibrationError("no calibration loaded; call connect() first")
        return self._calibration

    def _read(self, register: str, retries: int | None = None) -> dict[int, int]:
        try:
            values = self._bus.sync_read(self._model.motor_ids, register, retries=retries)
        except CommunicationError as exc:
            self._health.latch(exc)
            self._failures += 1
            if self._failures >= self._limits.max_consecutive_failures:
                self._emergency_stop()
                raise WatchdogTimeout(
                    f"{self._failures} consecutive bus failures while reading {register!r}; "
                    "torque disabled"
                ) from exc
            raise
        self._failures = 0
        return values

    def read_once(
        self,
        health: bool | None = None,
        *,
        retries: int | None = None,
        health_period_s: float | None = None,
    ) -> ArmState:
        """Read the arm once.

        Args:
            health: ``True`` refreshes all health registers at once, ``False`` reuses the
                cache, ``None`` (default) refreshes one register per tick (round-robin) once the
                cache is older than ``health_period_s``. The hot path (``False``) is a single
                sync read, and even the refreshing path adds only one extra transaction per
                tick — that is what keeps a 200 Hz loop inside its 5 ms budget.
            retries: Per-call override of the bus retry count, for hot paths that prefer a
                short timeout with several fast attempts.
            health_period_s: Override the arm's health refresh interval for this call; only
                used when ``health is None``.

        Returns:
            A fresh :class:`~pysoarm.state.ArmState`.

        Raises:
            CommunicationError: A transaction failed (counted by the watchdog).
            WatchdogTimeout: Too many consecutive failures; torque is already disabled.
            CalibrationError: No calibration available.
        """
        calibration = self._require_calibration()
        ids = self._model.motor_ids
        positions = self._read("present_position", retries)
        now = self._clock()
        if health is True:
            self._health.refresh_all(self._read, retries)
            now = self._clock()
        elif health is None and now - self._health.refreshed_at >= (
            self._health_period_s if health_period_s is None else health_period_s
        ):
            self._health.refresh_incremental(self._read, retries)
            now = self._clock()
        health_age = self._health.age(now)
        speed = self._health.speed_raw
        current = self._health.current
        temperature = self._health.temperature_c
        voltage = self._health.voltage_v
        state = ArmState(
            q_deg=tuple(
                calibration.for_joint(joint).raw_to_deg(positions[mid])
                for joint, mid in zip(self._model.joints, ids, strict=True)
            ),
            speed_raw=tuple(int(value) for value in speed),
            current=tuple(int(value) for value in current),
            temperature_c=tuple(int(value) for value in temperature),
            voltage_v=tuple(float(value) for value in voltage),
            t=now,
            consecutive_failures=self._failures,
            health_age_s=health_age,
        )
        self._state = state
        return state

    # -- writing ----------------------------------------------------------- #
    def write_goal(
        self,
        q_deg: Sequence[float],
        *,
        step_deg: float | None = None,
        retries: int | None = None,
    ) -> tuple[float, ...]:
        """Send a joint goal, after the safety guards.

        Args:
            q_deg: Requested goal in degrees, in the model's joint order.
            step_deg: Per-tick step limit override; defaults to
                ``limits.step_for(last loop rate) * speed_scale``.
            retries: Per-call override of the bus retry count.

        Returns:
            The goal actually written — clipped/clamped values are visible here.

        Raises:
            ConfigurationError: Wrong number of joints.
            SafetyViolation: Non-finite goal, over-temperature or undervoltage.
            WatchdogTimeout: Too many consecutive read failures.
            CommunicationError: The write failed.
        """
        if self._role == "leader":
            raise ConfigurationError("leader arms are read-only; no goal is written")
        calibration = self._require_calibration()
        state = self._state
        if state is None:
            raise ConfigurationError("no state yet; call read_once() before write_goal()")
        if step_deg is None:
            step_deg = self._limits.step_for(self._last_hz) * self._speed_scale
        report: SafetyReport = apply_guards(
            q_deg,
            state=state,
            calibration=calibration,
            model=self._model,
            limits=self._limits,
            step_deg=step_deg,
        )
        raw = calibration.deg_to_raw(dict(zip(self._model.joints, report.goal_deg, strict=True)))
        values: Mapping[int, int] = {
            self._model.id_of(joint): raw[joint] for joint in self._model.joints
        }
        try:
            self._bus.sync_write("goal_position", values, retries=retries)
        except CommunicationError as exc:
            self._failures += 1
            if self._failures >= self._limits.max_consecutive_failures:
                self._emergency_stop()
                raise WatchdogTimeout(
                    f"{self._failures} consecutive bus failures while writing a goal; "
                    "torque disabled"
                ) from exc
            raise
        return report.goal_deg

    def _ids_for(self, joints: Sequence[str | int] | None) -> list[int]:
        """Resolve joint names/ids to motor ids, validating every entry."""
        if joints is None:
            return list(self._model.motor_ids)
        selected: set[int] = set()
        for joint in joints:
            mid = self._model.id_of(joint) if isinstance(joint, str) else int(joint)
            if mid not in self._model.motor_ids:
                raise ConfigurationError(f"Motor {mid} is not part of model {self._model.name!r}")
            selected.add(mid)
        # Canonical model order, so mappings are stable regardless of argument order.
        return [mid for mid in self._model.motor_ids if mid in selected]

    def enable_torque(self) -> None:
        """Enable torque on every motor, holding the last commanded goal.

        Raises:
            BusConnectionError: If the bus is not open.
            ConfigurationError: On a read-only (leader) arm.
        """
        if self._role == "leader":
            raise ConfigurationError("leader arms are read-only; torque is never enabled")
        if not self._bus.is_open:
            raise BusConnectionError("bus is not open; call connect() first")
        self._bus.enable_torque(self._model.motor_ids)

    def disable_torque(self) -> None:
        """Disable torque on every motor (the arm goes limp).

        Raises:
            BusConnectionError: If the bus is not open.
        """
        if not self._bus.is_open:
            raise BusConnectionError("bus is not open; call connect() first")
        self._bus.disable_torque(self._model.motor_ids)

    def enable(self, joints: Sequence[str | int] | None = None) -> None:
        """Enable torque on a subset of joints (default: all).

        Args:
            joints: Joint names (``"gripper"``) or motor ids (``6``).

        Raises:
            ConfigurationError: Unknown joint/id, or a read-only arm.
            BusConnectionError: If the bus is not open.
        """
        if self._role == "leader":
            raise ConfigurationError("leader arms are read-only; torque is never enabled")
        if not self._bus.is_open:
            raise BusConnectionError("bus is not open; call connect() first")
        self._bus.enable_torque(self._ids_for(joints))

    def disable(self, joints: Sequence[str | int] | None = None) -> None:
        """Disable torque on a subset of joints (default: all).

        Raises:
            ConfigurationError: Unknown joint/id.
            BusConnectionError: If the bus is not open.
        """
        if not self._bus.is_open:
            raise BusConnectionError("bus is not open; call connect() first")
        self._bus.disable_torque(self._ids_for(joints))

    def is_enabled(self, joints: Sequence[str | int] | None = None) -> dict[str, bool]:
        """Read the ``torque_enable`` register of a subset of joints.

        Args:
            joints: Joint names or motor ids; defaults to every joint.

        Returns:
            Mapping ``joint -> torque_enable`` in the model's joint order.
        """
        ids = self._ids_for(joints)
        values = self._bus.sync_read(ids, "torque_enable")
        return {self._model.joint_of(mid): bool(values[mid]) for mid in ids}

    def emergency_stop(self) -> None:
        """Cut torque without closing the bus.

        This is the in-session stop: it is idempotent, never raises, and keeps the
        connection alive so the caller can inspect state, clear a fault and re-enable.
        :meth:`stop` remains the resource-releasing variant used by ``with``.
        """
        with contextlib.suppress(Exception):
            self._bus.disable_torque(self._model.motor_ids)

    def reset_faults(self, joints: Sequence[str | int] | None = None) -> dict[str, bool]:
        """Clear latched faults by cycling torque off and back on.

        The STS3215 has no dedicated clear-error register; cycling ``torque_enable`` is the
        documented recovery. Joints that were enabled before the call are re-enabled, so a
        successfully cleared fault does not leave the arm limp. The previous enable state is
        returned for inspection.

        Args:
            joints: Joint names or motor ids; defaults to every joint.

        Returns:
            Mapping ``joint -> was_enabled``.
        """
        ids = self._ids_for(joints)
        before = self.is_enabled(ids)
        self._bus.disable_torque(ids)
        if self._role == "follower":
            re_enable = [mid for mid in ids if before[self._model.joint_of(mid)]]
            if re_enable:
                self._bus.enable_torque(re_enable)
        return before

    def hold(self) -> ArmState:
        """Read the arm and immediately command the measured pose (a safe hold)."""
        state = self.read_once()
        self.write_goal(state.q_deg)
        return state

    def _emergency_stop(self) -> None:
        self.emergency_stop()

    # -- diagnostics / tuning ---------------------------------------------- #
    def health(self, *, retries: int | None = None) -> dict[str, MotorHealth]:
        """Read every health register and return a structured per-motor view.

        Alarm bits observed in a status packet are latched in the returned flags;
        ``"voltage"`` and ``"overheat"`` are additionally derived from the measurement and
        the active :class:`~pysoarm.safety.SafetyLimits`. Diagnostics only — the flags do
        not change the control-path guards.

        Args:
            retries: Per-call override of the bus retry count.

        Returns:
            Mapping ``joint -> MotorHealth`` in the model's joint order.
        """
        self._health.refresh_all(self._read, retries)
        return build_health(
            self._model,
            self._health,
            temp_warn_c=self._limits.temp_warn_c,
            voltage_min_v=self._limits.voltage_min_v,
        )

    def set_speed_scale(self, scale: float) -> float:
        """Scale every per-tick step limit, for the first cautious run on hardware.

        Args:
            scale: Multiplier in ``(0, 1]``.

        Returns:
            The scale now in effect.

        Raises:
            ConfigurationError: If ``scale`` is outside ``(0, 1]``.
        """
        if not 0.0 < scale <= 1.0:
            raise ConfigurationError("speed scale must be in (0, 1]")
        self._speed_scale = scale
        return self._speed_scale

    def motor_config(self, joint: str) -> dict[str, int]:
        """Read the writable tuning registers of one joint.

        Args:
            joint: Joint name.

        Returns:
            Mapping ``register -> value`` for :data:`MOTOR_CONFIG_REGISTERS`.
        """
        return read_motor_config(self._bus, self._model, joint)

    def set_motor_config(self, joint: str, **values: int) -> dict[str, int]:
        """Write tuning registers of one joint.

        Args:
            joint: Joint name.
            **values: Register/value pairs, each in :data:`MOTOR_CONFIG_REGISTERS`.

        Returns:
            The values written.

        Raises:
            ConfigurationError: On an unknown register or a read-only arm.
        """
        if self._role == "leader":
            raise ConfigurationError("leader arms are read-only; motor config is unchanged")
        return write_motor_config(self._bus, self._model, joint, values)

    def raw_position(self, joint: str) -> int:
        """Read the raw ``present_position`` of one joint (ticks)."""
        return self._bus.sync_read(self._model.motor_ids, "present_position")[
            self._model.id_of(joint)
        ]

    def write_calibration(self, calibration: Calibration) -> None:
        """Persist a calibration to the motors and use it immediately.

        Args:
            calibration: Calibration matching this arm's model.

        Raises:
            CalibrationError: If it does not match the model.
        """
        write_calibration(self._bus, self._model, calibration)
        self._calibration = calibration

    @property
    def gripper(self) -> Gripper:
        """A thin :class:`~pysoarm.gripper.Gripper` facade over the sixth joint.

        Raises:
            ConfigurationError: If the model has no gripper joint.
        """
        if self._model.gripper_joint is None:
            raise ConfigurationError(f"model {self._model.name!r} has no gripper")
        if self._gripper is None:
            self._gripper = Gripper(self)
        return self._gripper

    # -- motion ------------------------------------------------------------ #
    def _build_trajectory(
        self, start: tuple[float, ...], end: Sequence[float], duration: float, profile: str
    ) -> Trajectory:
        target = tuple(float(value) for value in end)
        if len(target) != self._model.joint_count:
            raise ConfigurationError(
                f"Expected {self._model.joint_count} joints, got {len(target)}"
            )
        if profile == "minjerk":
            return MinimumJerk(start=start, end=target, duration=duration)
        if profile == "linear":
            return LinearRamp(start=start, end=target, duration=duration)
        raise ConfigurationError(f"Unknown profile {profile!r}; expected 'minjerk' or 'linear'")

    def _require_kinematics(self) -> Kinematics:
        if self._kinematics is None:
            raise ConfigurationError(
                "no kinematics configured; pass kinematics=PlacoKinematics(urdf) to SOArm"
            )
        return self._kinematics

    def forward(self, q_deg: Sequence[float] | None = None) -> Pose:
        """End-effector pose for ``q_deg`` (default: the last measured pose).

        Args:
            q_deg: Joint values in degrees; ``None`` uses the most recent state.

        Returns:
            The end-effector :class:`~pysoarm.kinematics.base.Pose`.

        Raises:
            ConfigurationError: No kinematics solver, no state, or a wrong joint count.
        """
        kinematics = self._require_kinematics()
        if q_deg is None:
            if self._state is None:
                raise ConfigurationError("no state yet; call read_once() or pass q_deg")
            values = self._state.q_deg
        else:
            values = tuple(float(value) for value in q_deg)
        if len(values) != self._model.joint_count:
            raise ConfigurationError(
                f"Expected {self._model.joint_count} joint values, got {len(values)}"
            )
        return kinematics.forward(values)

    def move_to_pose(
        self,
        pose: Pose,
        *,
        duration: float = 2.0,
        hz: float | None = None,
        seed_deg: Sequence[float] | None = None,
        max_iters: int | None = None,
    ) -> ArmState:
        """Move the end effector to ``pose``: solve IK, then run a joint-space move.

        The IK solve happens **before** any goal is written, so an unreachable target leaves
        the arm exactly where it was (:class:`~pysoarm.exceptions.NotReachableError`). The
        resulting joint goal still passes every safety guard through :meth:`write_goal`.

        Args:
            pose: Target end-effector pose.
            duration: Motion duration in seconds.
            hz: Loop rate for the joint-space move; defaults to 200 Hz.
            seed_deg: IK seed; defaults to the current measured joints.
            max_iters: Override the solver's iteration budget.

        Returns:
            The state measured after the move.

        Raises:
            ConfigurationError: No kinematics solver or mismatched joint count.
            NotReachableError: IK did not converge; the arm was not commanded.
        """
        kinematics = self._require_kinematics()
        state = self._state if self._state is not None else self.read_once()
        seed = state.q_deg if seed_deg is None else tuple(float(v) for v in seed_deg)
        solution = kinematics.inverse(pose, seed_deg=seed, max_iters=max_iters)
        if len(solution) != self._model.joint_count:
            raise ConfigurationError(
                f"Kinematics returned {len(solution)} joints, model "
                f"{self._model.name!r} has {self._model.joint_count}"
            )
        return self.move_to(solution, duration=duration, hz=hz)

    def move_to(
        self,
        q_deg: Sequence[float],
        *,
        duration: float = 2.0,
        profile: str = "minjerk",
        hz: float | None = None,
    ) -> ArmState:
        """Move to a joint goal along a time-parameterised profile, then return the state.

        Args:
            q_deg: Target in degrees, in the model's joint order.
            duration: Motion duration in seconds.
            profile: ``"minjerk"`` (default) or ``"linear"``.
            hz: Loop rate; defaults to 200 Hz.

        Returns:
            The state measured after the move settles.

        Raises:
            ConfigurationError: Bad target length or unknown profile.
            SafetyViolation: A guard refused the goal (see :meth:`write_goal`).
        """
        state = self._state if self._state is not None else self.read_once()
        trajectory = self._build_trajectory(state.q_deg, q_deg, duration, profile)
        rate = DEFAULT_LOOP_HZ if hz is None else hz
        if rate <= 0:
            raise ConfigurationError("hz must be positive")
        self._last_hz = rate
        period = 1.0 / rate
        t0 = self._clock()
        tick = 0
        while True:
            elapsed = self._clock() - t0
            if elapsed >= duration:
                break
            self.write_goal(trajectory.sample(elapsed))
            self._state = self.read_once(health=False)
            deadline = t0 + (tick + 1) * period
            slack = deadline - self._clock()
            if slack > 0:
                self._sleep(slack)
            tick += 1
        self.write_goal(trajectory.sample(duration))
        if _SETTLE_S > 0:
            self._sleep(_SETTLE_S)
        self._state = self.read_once(health=False)
        return self._state

    def run_control_loop(
        self,
        callback: Callable[[ArmState, float], Sequence[float] | None],
        *,
        hz: float = DEFAULT_LOOP_HZ,
        duration: float | None = None,
        read_state_every: int | None = 1,
        health_period_s: float | None = None,
        on_error: ErrorPolicy = "abort",
        max_consecutive_errors: int = 10,
        retries: int | None = None,
    ) -> LoopStats:
        """Run a fixed-rate control loop.

        Each tick: read a state, call ``callback(state, elapsed_s)``, write the returned goal
        (``None`` means "hold, send nothing"). Deadlines are absolute, so jitter does not
        accumulate; overruns are counted instead of silently shortening the period. The
        timing and error policy live in :mod:`pysoarm.loop`; this method binds them to the
        arm's reads, writes, clock and sleep.

        Args:
            callback: ``(state, elapsed_s) -> goal_deg | None``.
            hz: Target loop rate.
            duration: Stop after this many seconds; ``None`` runs until ``KeyboardInterrupt``.
            read_state_every: Read a fresh state every ``N`` ticks. ``0`` or ``None`` never
                reads inside the loop and reuses :attr:`state` (the caller then owns
                ``read_once()``). ``N > 1`` widens the per-step guard because the state may
                be up to ``N`` ticks old.
            health_period_s: ``None`` (default) disables background health refresh in the
                hot path. A float re-enables it through the arm's round-robin refresh.
            on_error: What to do when a read, callback or write raises:
                ``"abort"`` (default) re-raises, ``"hold"`` leaves the last goal in place and
                continues, ``"skip"`` discards the tick and continues.
            max_consecutive_errors: Abort after this many consecutive handled errors, so a
                dead bus cannot spin forever.
            retries: Per-call bus retry override for the hot path.

        Returns:
            :class:`LoopStats` with ticks, overruns, errors and latency percentiles.

        Raises:
            ConfigurationError: Non-positive ``hz``, a negative ``read_state_every``, an
                unknown error policy, or ``read_state_every`` in ``(0, None)`` without a
                cached state.
            PysoarmError: The original error when ``on_error="abort"``.
        """
        validate_loop_options(
            hz=hz,
            read_state_every=read_state_every,
            on_error=on_error,
            max_consecutive_errors=max_consecutive_errors,
            initial_state=self._state,
        )
        self._last_hz = hz
        return run_loop(
            callback=callback,
            read_state=lambda: self.read_once(
                health=None if health_period_s is not None else False,
                retries=retries,
                health_period_s=health_period_s,
            ),
            write_goal=lambda goal: self.write_goal(goal, retries=retries),
            initial_state=self._state,
            hz=hz,
            duration=duration,
            read_state_every=read_state_every,
            on_error=on_error,
            max_consecutive_errors=max_consecutive_errors,
            clock=self._clock,
            sleep=self._sleep,
        )
