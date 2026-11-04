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
import math
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from pysoarm.bus import Backend, make_bus
from pysoarm.bus.base import Bus
from pysoarm.calibration import Calibration, JointCalibration
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
    VOLTAGE_LSB_V,
)
from pysoarm.exceptions import (
    BusConnectionError,
    CalibrationError,
    CommunicationError,
    ConfigurationError,
    WatchdogTimeout,
)
from pysoarm.models.base import RobotModel, get_model
from pysoarm.safety import SafetyLimits, SafetyReport, apply_guards
from pysoarm.state import ArmState
from pysoarm.trajectory import LinearRamp, MinimumJerk, Trajectory

__all__ = ["LoopStats", "SOArm"]

#: Extra settle time after a blocking move, so the reported state reflects the endpoint.
_SETTLE_S = 0.02
#: Health registers, refreshed one per tick when the cache goes stale. Spread over four ticks
#: (~+0.36 ms each) so no single control tick pays for a full health read.
_HEALTH_REGISTERS: tuple[str, ...] = (
    "present_speed",
    "present_current",
    "present_temperature",
    "present_voltage",
)


@dataclass(frozen=True, slots=True)
class LoopStats:
    """Summary of a finished control loop.

    Args:
        ticks: Number of completed ticks.
        overruns: Ticks that could not be finished within the period.
        elapsed_s: Wall-clock duration of the loop.
    """

    ticks: int
    overruns: int
    elapsed_s: float

    @property
    def mean_period_s(self) -> float:
        """Average tick period in seconds (``0.0`` for an empty loop)."""
        return self.elapsed_s / self.ticks if self.ticks else 0.0

    @property
    def mean_hz(self) -> float:
        """Average tick rate in Hz (``0.0`` for an empty loop)."""
        return 1.0 / self.mean_period_s if self.mean_period_s > 0 else 0.0

    @property
    def overrun_ratio(self) -> float:
        """Fraction of ticks that overran their budget."""
        return self.overruns / self.ticks if self.ticks else 0.0


class SOArm:
    """One SO-100/SO-101 arm on a serial bus.

    Args:
        port: Serial device, e.g. ``"/dev/so101_follower"``. Required unless ``bus`` is given.
        bus: Pre-built transport (tests inject :class:`~pysoarm.bus.fake.FakeBus`).
        backend: Transport to build when ``bus`` is not given: ``"scs"`` (default),
            ``"rustypot"`` (optional extra, ~2x faster reads) or ``"auto"``.
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
        health_period_s: How often to refresh temperature/voltage/current.
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
        backend: Backend = "scs",
        model: str | RobotModel = "so101",
        calibration: Calibration | str | Path | None = None,
        limits: SafetyLimits | None = None,
        baudrate: int | None = None,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        retries: int = DEFAULT_RETRIES,
        configure_motors: bool = True,
        enable_torque: bool = True,
        health_period_s: float = DEFAULT_HEALTH_PERIOD_S,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if bus is None and port is None:
            raise ConfigurationError("pass either port= or bus=")
        if health_period_s < 0:
            raise ConfigurationError("health_period_s must be non-negative")
        self._model = get_model(model) if isinstance(model, str) else model
        self._limits = limits if limits is not None else SafetyLimits()
        self._backend: Backend = backend
        if bus is not None:
            self._bus = bus
        else:
            assert port is not None  # narrowed by the check above
            self._backend = backend
            self._bus = make_bus(
                port,
                self._model.motor_ids,
                backend=backend,
                servo_model=self._model.servo_model,
                baudrate=baudrate or self._model.baudrate,
                timeout_s=timeout_s,
                retries=retries,
            )
        self._calibration = self._load_calibration(calibration)
        self._configure_motors = configure_motors
        self._enable_torque = enable_torque
        self._health_period_s = health_period_s
        self._clock = clock
        self._sleep = sleep
        self._connected = False
        self._state: ArmState | None = None
        self._failures = 0
        self._health: dict[str, tuple[float, ...] | tuple[int, ...]] = {}
        self._health_t = -math.inf
        self._health_index = 0

    # -- properties -------------------------------------------------------- #
    @property
    def backend(self) -> Backend:
        """Transport selected at construction."""
        return self._backend

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
    def connect(self) -> None:
        """Open the bus, configure the motors, hold the current pose, enable torque.

        The sequence is deliberately conservative: torque is off while the motors are
        configured, the measured pose is written back as the first goal, and only then is
        torque enabled.

        Raises:
            BusConnectionError: Port unavailable or a motor did not answer.
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
            if self._calibration is None:
                self._calibration = self._read_calibration_from_motors()
            self._calibration.validate(self._model)
            self._bus.disable_torque(self._model.motor_ids)
            if self._configure_motors:
                self._configure()
            state = self.read_once(health=True)
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
        for register, value in constants:
            self._bus.sync_write(register, dict.fromkeys(ids, value))

        if self._calibration is not None:
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
        if gripper_id is not None:
            for register, value in self._model.gripper_register_defaults:
                self._bus.write(gripper_id, register, value)

    def _read_calibration_from_motors(self) -> Calibration:
        ids = self._model.motor_ids
        mins = self._bus.sync_read(ids, "min_position_limit")
        maxes = self._bus.sync_read(ids, "max_position_limit")
        offsets = self._bus.sync_read(ids, "homing_offset")
        joints: dict[str, JointCalibration] = {}
        for joint, mid in zip(self._model.joints, ids, strict=True):
            if mins[mid] >= maxes[mid]:
                raise CalibrationError(
                    f"Motor {mid} ({joint}) reports no travel "
                    f"([{mins[mid]}, {maxes[mid]}]); the arm is not calibrated. "
                    "Run `lerobot-calibrate` or pass --calibration."
                )
            joints[joint] = JointCalibration(
                motor_id=mid,
                range_min=mins[mid],
                range_max=maxes[mid],
                homing_offset=offsets[mid],
            )
        return Calibration(joints)

    # -- reading ----------------------------------------------------------- #
    def _require_calibration(self) -> Calibration:
        if self._calibration is None:
            raise CalibrationError("no calibration loaded; call connect() first")
        return self._calibration

    def _read(self, register: str) -> dict[int, int]:
        try:
            values = self._bus.sync_read(self._model.motor_ids, register)
        except CommunicationError as exc:
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

    def _read_health_register(self, register: str) -> None:
        values = self._read(register)
        ids = self._model.motor_ids
        if register == "present_voltage":
            self._health["voltage_v"] = tuple(values[mid] * VOLTAGE_LSB_V for mid in ids)
        elif register == "present_temperature":
            self._health["temperature_c"] = tuple(values[mid] for mid in ids)
        elif register == "present_speed":
            self._health["speed_raw"] = tuple(values[mid] for mid in ids)
        else:
            self._health["current"] = tuple(values[mid] for mid in ids)

    def _refresh_health(self) -> None:
        for register in _HEALTH_REGISTERS:
            self._read_health_register(register)
        self._health_index = 0
        self._health_t = self._clock()

    def _refresh_health_incrementally(self) -> None:
        """Refresh one health register; the cache counts as fresh after four ticks."""
        register = _HEALTH_REGISTERS[self._health_index % len(_HEALTH_REGISTERS)]
        self._read_health_register(register)
        self._health_index += 1
        if self._health_index % len(_HEALTH_REGISTERS) == 0:
            self._health_t = self._clock()

    def read_once(self, health: bool | None = None) -> ArmState:
        """Read the arm once.

        Args:
            health: ``True`` refreshes all health registers at once, ``False`` reuses the
                cache, ``None`` (default) refreshes one register per tick (round-robin) once the
                cache is older than ``health_period_s``. The hot path (``False``) is a single
                sync read, and even the refreshing path adds only one extra transaction per
                tick — that is what keeps a 200 Hz loop inside its 5 ms budget.

        Returns:
            A fresh :class:`~pysoarm.state.ArmState`.

        Raises:
            CommunicationError: A transaction failed (counted by the watchdog).
            WatchdogTimeout: Too many consecutive failures; torque is already disabled.
            CalibrationError: No calibration available.
        """
        calibration = self._require_calibration()
        ids = self._model.motor_ids
        positions = self._read("present_position")
        now = self._clock()
        if health is True:
            self._refresh_health()
            now = self._clock()
        elif health is None and now - self._health_t >= self._health_period_s:
            self._refresh_health_incrementally()
            now = self._clock()

        speed = self._health.get("speed_raw", tuple(0 for _ in ids))
        current = self._health.get("current", tuple(0 for _ in ids))
        temperature = self._health.get("temperature_c", tuple(0.0 for _ in ids))
        voltage = self._health.get("voltage_v", tuple(0.0 for _ in ids))
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
        )
        self._state = state
        return state

    # -- writing ----------------------------------------------------------- #
    def write_goal(self, q_deg: Sequence[float]) -> tuple[float, ...]:
        """Send a joint goal, after the safety guards.

        Args:
            q_deg: Requested goal in degrees, in the model's joint order.

        Returns:
            The goal actually written — clipped/clamped values are visible here.

        Raises:
            ConfigurationError: Wrong number of joints.
            SafetyViolation: Non-finite goal, over-temperature or undervoltage.
            WatchdogTimeout: Too many consecutive read failures.
            CommunicationError: The write failed.
        """
        calibration = self._require_calibration()
        state = self._state
        if state is None:
            raise ConfigurationError("no state yet; call read_once() before write_goal()")
        report: SafetyReport = apply_guards(
            q_deg,
            state=state,
            calibration=calibration,
            model=self._model,
            limits=self._limits,
        )
        raw = calibration.deg_to_raw(dict(zip(self._model.joints, report.goal_deg, strict=True)))
        values: Mapping[int, int] = {
            self._model.id_of(joint): raw[joint] for joint in self._model.joints
        }
        try:
            self._bus.sync_write("goal_position", values)
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

    def hold(self) -> ArmState:
        """Read the arm and immediately command the measured pose (a safe hold)."""
        state = self.read_once()
        self.write_goal(state.q_deg)
        return state

    def _emergency_stop(self) -> None:
        with contextlib.suppress(Exception):
            self._bus.disable_torque(self._model.motor_ids)

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
    ) -> LoopStats:
        """Run a fixed-rate control loop.

        Each tick: read one state, call ``callback(state, elapsed_s)``, write the returned goal
        (``None`` means "hold, send nothing"). Deadlines are absolute, so jitter does not
        accumulate; overruns are counted instead of silently shortening the period.

        Args:
            callback: ``(state, elapsed_s) -> goal_deg | None``.
            hz: Target loop rate.
            duration: Stop after this many seconds; ``None`` runs until ``KeyboardInterrupt``.

        Returns:
            :class:`LoopStats` with tick count and overruns.

        Raises:
            ConfigurationError: Non-positive ``hz``.
        """
        if hz <= 0:
            raise ConfigurationError("hz must be positive")
        period = 1.0 / hz
        t0 = self._clock()
        ticks = 0
        overruns = 0
        try:
            while True:
                now = self._clock()
                elapsed = now - t0
                if duration is not None and elapsed >= duration:
                    break
                state = self.read_once(health=None)
                goal = callback(state, elapsed)
                if goal is not None:
                    self.write_goal(goal)
                deadline = t0 + (ticks + 1) * period
                slack = deadline - self._clock()
                if slack > 0:
                    self._sleep(slack)
                else:
                    overruns += 1
                ticks += 1
        except KeyboardInterrupt:
            pass
        return LoopStats(ticks=ticks, overruns=overruns, elapsed_s=self._clock() - t0)
