"""Calibration: homing offsets, joint ranges and the degrees <-> tick conversion.

The on-disk format is intentionally LeRobot-compatible, so an arm that was already calibrated
by LeRobot can be driven by pysoarm without re-calibration::

    {
      "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": -1902,
                       "range_min": 732, "range_max": 3380},
      ...
    }

Conversions (the STS3215 has 4096 ticks per turn, so 4095 is the largest representable raw
value)::

    raw   = round(q_deg * 4095 / 360 + (range_min + range_max) / 2)
    q_deg = (raw - (range_min + range_max) / 2) * 360 / 4095

``drive_mode == 1`` marks a mirrored mounting: the sign is flipped in both directions. LeRobot
ignores ``drive_mode`` in degrees mode; pysoarm applies it, because a mirrored joint that is
not flipped moves the wrong way. All SO-101 factory calibrations use ``drive_mode = 0``.
"""

from __future__ import annotations

import contextlib
import json
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

from pysoarm.bus.base import Bus
from pysoarm.constants import RESOLUTION_MAX
from pysoarm.exceptions import CalibrationError, ConfigurationError
from pysoarm.models.base import RobotModel

__all__ = [
    "FULL_TURN_JOINTS",
    "Calibration",
    "JointCalibration",
    "calibrate",
    "read_calibration",
    "record_ranges",
    "reset_calibration",
    "set_half_turn_homings",
    "write_calibration",
]

#: The reference arm uses 4095 as the full-scale raw value (``resolution - 1``).
_FULL_SCALE = float(RESOLUTION_MAX)
#: Raw position the servo reports after a correct half-turn homing. The 11-bit
#: sign-magnitude register cannot represent 2048, so 2047 is the canonical value
#: (see the improvement report, P0-3).
_HALF_TURN_RAW = RESOLUTION_MAX // 2
#: Largest homing offset the 11-bit sign-magnitude register can carry.
_HOMING_OFFSET_MAX = (1 << 11) - 1
#: Joints that rotate continuously and are calibrated over the whole encoder range.
FULL_TURN_JOINTS: tuple[str, ...] = ("wrist_roll",)


@dataclass(frozen=True, slots=True)
class JointCalibration:
    """Calibration of a single joint.

    Args:
        motor_id: Servo id this calibration belongs to.
        range_min: Raw tick at the lower mechanical end of travel.
        range_max: Raw tick at the upper mechanical end of travel.
        homing_offset: Motor-side homing offset (recorded, not applied by pysoarm).
        drive_mode: ``1`` for a mirrored mounting (sign flipped), else ``0``.

    Raises:
        CalibrationError: If the range is degenerate, out of the encodable range, or if the
            homing offset does not fit the servo's 11-bit sign-magnitude register.
    """

    motor_id: int
    range_min: int
    range_max: int
    homing_offset: int = 0
    drive_mode: int = 0

    def __post_init__(self) -> None:
        if not 0 <= self.range_min < self.range_max <= RESOLUTION_MAX:
            raise CalibrationError(
                f"Motor {self.motor_id}: need 0 <= range_min < range_max <= {RESOLUTION_MAX}, "
                f"got [{self.range_min}, {self.range_max}]"
            )
        if self.drive_mode not in (0, 1):
            raise CalibrationError(
                f"Motor {self.motor_id}: drive_mode must be 0 or 1, got {self.drive_mode}"
            )
        if abs(self.homing_offset) > _HOMING_OFFSET_MAX:
            raise CalibrationError(
                f"Motor {self.motor_id}: homing_offset {self.homing_offset} does not fit "
                f"the +/-{_HOMING_OFFSET_MAX} register"
            )

    @property
    def midpoint(self) -> float:
        """Raw tick that corresponds to 0 degrees."""
        return (self.range_min + self.range_max) / 2.0

    @property
    def span_deg(self) -> float:
        """Usable travel in degrees."""
        return (self.range_max - self.range_min) * 360.0 / _FULL_SCALE

    def raw_to_deg(self, raw: int) -> float:
        """Convert a raw tick to degrees (no clamping)."""
        deg = (raw - self.midpoint) * 360.0 / _FULL_SCALE
        return -deg if self.drive_mode else deg

    def deg_to_raw(self, deg: float) -> int:
        """Convert degrees to the nearest raw tick, clamped to the encodable range."""
        value = -deg if self.drive_mode else deg
        raw = round(value * _FULL_SCALE / 360.0 + self.midpoint)
        return min(RESOLUTION_MAX, max(0, raw))

    def clamp_deg(self, deg: float) -> float:
        """Clamp a degree value to the calibrated travel."""
        lo, hi = self.raw_to_deg(self.range_min), self.raw_to_deg(self.range_max)
        return min(max(deg, min(lo, hi)), max(lo, hi))

    def as_dict(self) -> dict[str, int]:
        """JSON-serialisable view in LeRobot's field order."""
        return {
            "id": self.motor_id,
            "drive_mode": self.drive_mode,
            "homing_offset": self.homing_offset,
            "range_min": self.range_min,
            "range_max": self.range_max,
        }

    @classmethod
    def from_dict(cls, motor_id: int, data: Mapping[str, object]) -> JointCalibration:
        """Build from a LeRobot-style mapping (``id`` is taken from the entry if present)."""

        def _int(key: str, default: int | None = None) -> int:
            raw = data.get(key, default)
            if raw is None:
                raise CalibrationError(f"Missing {key!r} in calibration for motor {motor_id}")
            if isinstance(raw, bool) or not isinstance(raw, int | str):
                raise CalibrationError(
                    f"Invalid {key!r} for motor {motor_id}: {raw!r} is not an integer"
                )
            try:
                return int(raw)
            except ValueError as exc:
                raise CalibrationError(
                    f"Invalid {key!r} for motor {motor_id}: {raw!r} is not an integer"
                ) from exc

        return cls(
            motor_id=_int("id", motor_id),
            range_min=_int("range_min"),
            range_max=_int("range_max"),
            homing_offset=_int("homing_offset", 0),
            drive_mode=_int("drive_mode", 0),
        )


@dataclass(frozen=True, slots=True)
class Calibration:
    """Calibration of a whole arm, keyed by joint name.

    Args:
        joints: Mapping of joint name to :class:`JointCalibration`.
    """

    joints: Mapping[str, JointCalibration]

    def __post_init__(self) -> None:
        object.__setattr__(self, "joints", MappingProxyType(dict(self.joints)))

    # -- construction ------------------------------------------------------ #
    @classmethod
    def from_dict(cls, data: Mapping[str, Mapping[str, object]]) -> Calibration:
        """Build from a LeRobot-style mapping of joint name to entry.

        Raises:
            CalibrationError: If the data is malformed.
        """
        if not data:
            raise CalibrationError("Calibration file is empty")
        return cls({joint: JointCalibration.from_dict(0, entry) for joint, entry in data.items()})

    @classmethod
    def from_json(cls, path: str | Path) -> Calibration:
        """Load a LeRobot-compatible calibration JSON file.

        Raises:
            CalibrationError: If the file is missing or malformed.
        """
        p = Path(path).expanduser()
        try:
            raw = json.loads(p.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise CalibrationError(f"Calibration file not found: {p}") from exc
        except json.JSONDecodeError as exc:
            raise CalibrationError(f"Calibration file {p} is not valid JSON: {exc}") from exc
        if not isinstance(raw, dict):
            raise CalibrationError(f"Calibration file {p} must contain a JSON object")
        try:
            return cls.from_dict(raw)
        except TypeError as exc:
            raise CalibrationError(f"Calibration file {p} has an invalid structure: {exc}") from exc

    #: pysoarm's format is LeRobot's format; this alias documents the intent.
    from_lerobot = from_json

    # -- serialisation ----------------------------------------------------- #
    def to_dict(self) -> dict[str, dict[str, int]]:
        """JSON-serialisable view, keyed by joint name."""
        return {joint: cal.as_dict() for joint, cal in self.joints.items()}

    def to_json(self, path: str | Path) -> Path:
        """Write the calibration as LeRobot-compatible JSON; returns the path written."""
        p = Path(path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")
        return p

    to_lerobot = to_json

    # -- validation -------------------------------------------------------- #
    def validate(self, model: RobotModel) -> None:
        """Check the calibration against an arm model.

        Args:
            model: Arm the calibration will be used with.

        Raises:
            CalibrationError: On missing/extra joints or duplicated motor ids.
        """
        missing = [j for j in model.joints if j not in self.joints]
        extra = [j for j in self.joints if j not in model.joints]
        if missing or extra:
            raise CalibrationError(
                f"Calibration does not match model {model.name!r}: "
                f"missing={missing} unexpected={extra}"
            )
        seen: dict[int, str] = {}
        for joint, cal in self.joints.items():
            if cal.motor_id in seen:
                raise CalibrationError(
                    f"Motor {cal.motor_id} is assigned to both {seen[cal.motor_id]!r} and {joint!r}"
                )
            seen[cal.motor_id] = joint
            expected = model.id_of(joint)
            if cal.motor_id != expected:
                raise CalibrationError(
                    f"Joint {joint!r}: calibration is for motor {cal.motor_id}, "
                    f"model {model.name!r} expects {expected}"
                )

    def for_joint(self, joint: str) -> JointCalibration:
        """Calibration of one joint.

        Raises:
            CalibrationError: If the joint is unknown.
        """
        try:
            return self.joints[joint]
        except KeyError as exc:
            raise CalibrationError(
                f"No calibration for joint {joint!r}; known: {sorted(self.joints)}"
            ) from exc

    # -- conversions ------------------------------------------------------- #
    def raw_to_deg(self, raw: Mapping[str, int]) -> dict[str, float]:
        """Convert a mapping of raw ticks to degrees."""
        return {joint: self.for_joint(joint).raw_to_deg(value) for joint, value in raw.items()}

    def deg_to_raw(self, deg: Mapping[str, float]) -> dict[str, int]:
        """Convert a mapping of degrees to raw ticks (clamped to the encodable range)."""
        return {joint: self.for_joint(joint).deg_to_raw(value) for joint, value in deg.items()}

    def clamp_deg(self, deg: Mapping[str, float]) -> dict[str, float]:
        """Clamp a mapping of degrees to each joint's calibrated travel."""
        return {joint: self.for_joint(joint).clamp_deg(value) for joint, value in deg.items()}


# --------------------------------------------------------------------------- #
# Motor-side calibration flow (public since 0.1.1)
# --------------------------------------------------------------------------- #
def read_calibration(bus: Bus, model: RobotModel) -> Calibration:
    """Read the calibration stored in the motors.

    The motors keep three relevant registers: ``min_position_limit``,
    ``max_position_limit`` and ``homing_offset``. ``drive_mode`` has no register on the
    STS3215, so the returned calibration always uses ``drive_mode=0``; supply a JSON file
    with a different value if a joint is mounted mirrored.

    Args:
        bus: Open transport.
        model: Arm model that defines the joint order and motor ids.

    Returns:
        A :class:`Calibration` in the model's joint order.

    Raises:
        CalibrationError: A motor reports a degenerate travel range (the arm was never
            calibrated).
    """
    ids = model.motor_ids
    mins = bus.sync_read(ids, "min_position_limit")
    maxes = bus.sync_read(ids, "max_position_limit")
    offsets = bus.sync_read(ids, "homing_offset")
    joints: dict[str, JointCalibration] = {}
    for joint, mid in zip(model.joints, ids, strict=True):
        if mins[mid] >= maxes[mid]:
            raise CalibrationError(
                f"Motor {mid} ({joint}) reports no travel "
                f"([{mins[mid]}, {maxes[mid]}]); the arm is not calibrated. "
                "Run `pysoarm calibrate` or pass --calibration."
            )
        joints[joint] = JointCalibration(
            motor_id=mid,
            range_min=mins[mid],
            range_max=maxes[mid],
            homing_offset=offsets[mid],
        )
    return Calibration(joints)


def write_calibration(bus: Bus, model: RobotModel, calibration: Calibration) -> None:
    """Write a calibration to the motors.

    Args:
        bus: Open transport.
        model: Arm model that defines the joint order and motor ids.
        calibration: Calibration to persist.

    Raises:
        CalibrationError: If the calibration does not match ``model`` or a value does not
            fit its register.
    """
    calibration.validate(model)
    ids = model.motor_ids
    bus.sync_write(
        "min_position_limit",
        {mid: calibration.for_joint(model.joint_of(mid)).range_min for mid in ids},
    )
    bus.sync_write(
        "max_position_limit",
        {mid: calibration.for_joint(model.joint_of(mid)).range_max for mid in ids},
    )
    bus.sync_write(
        "homing_offset",
        {mid: calibration.for_joint(model.joint_of(mid)).homing_offset for mid in ids},
    )


def reset_calibration(bus: Bus, model: RobotModel) -> None:
    """Reset the motor-side calibration to a neutral state.

    Homing offsets are cleared and the travel is widened to the full encoder range. Torque
    is disabled first, so resetting never fights a holding servo.

    Args:
        bus: Open transport.
        model: Arm model.
    """
    ids = model.motor_ids
    bus.disable_torque(ids)
    bus.sync_write("homing_offset", dict.fromkeys(ids, 0))
    bus.sync_write("min_position_limit", dict.fromkeys(ids, 0))
    bus.sync_write("max_position_limit", dict.fromkeys(ids, RESOLUTION_MAX))


def set_half_turn_homings(bus: Bus, model: RobotModel) -> dict[str, int]:
    """Move the electrical zero of every joint to its current half-turn position.

    The Feetech convention is ``homing_offset = position - 2047`` (subtracted); the
    sign-magnitude register is clamped to ``+/-2047`` so a servo sitting at 4095 cannot
    produce the unrepresentable 2048.

    Args:
        bus: Open transport.
        model: Arm model.

    Returns:
        Mapping ``joint -> homing_offset`` actually written.

    Raises:
        ConfigurationError: If the bus rejects the write.
    """
    ids = model.motor_ids
    positions = bus.sync_read(ids, "present_position")
    offsets: dict[str, int] = {}
    for joint, mid in zip(model.joints, ids, strict=True):
        offset = positions[mid] - _HALF_TURN_RAW
        offsets[joint] = min(_HOMING_OFFSET_MAX, max(-_HOMING_OFFSET_MAX, offset))
    bus.sync_write("homing_offset", {mid: offsets[model.joint_of(mid)] for mid in ids})
    return offsets


def _enter_listener(prompt: str, input_fn: Callable[[str], str] = input) -> Callable[[], bool]:
    """Return a non-blocking ``stop`` predicate that turns true after Enter.

    ``input`` runs on a daemon thread so polling continues while the operator moves the
    joints. This works on Linux, macOS and Windows without ``select``/``msvcrt``.
    """
    pressed = threading.Event()

    def listen() -> None:
        with contextlib.suppress(EOFError, KeyboardInterrupt):
            input_fn(prompt)
        pressed.set()

    threading.Thread(target=listen, name="pysoarm-calibration", daemon=True).start()
    return pressed.is_set


def record_ranges(
    bus: Bus,
    model: RobotModel,
    *,
    stop: Callable[[], bool] | None = None,
    joints: Sequence[str] | None = None,
    poll_s: float = 0.02,
    min_span_ticks: int = 1,
    require_motion: bool = True,
    timeout_s: float | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    prompt: str = "Move every joint slowly through its full travel, then press Enter... ",
    input_fn: Callable[[str], str] = input,
) -> tuple[dict[str, int], dict[str, int]]:
    """Record the observed travel range of each joint.

    Args:
        bus: Open transport.
        model: Arm model.
        stop: Predicate polled after every sample; ``True`` ends the recording. Defaults to
            waiting for Enter (on a background thread, see :func:`_enter_listener`).
        joints: Subset of joint names to record; defaults to every joint of ``model``.
        poll_s: Sampling period in seconds.
        min_span_ticks: Minimum ``max - min`` considered as "moved".
        require_motion: Raise when a joint did not move at all.
        timeout_s: Optional safety timeout; recording stops after this many seconds.
        sleep: Sleep function, injectable for tests.
        clock: Monotonic clock, injectable for tests.
        prompt: Prompt shown by the default Enter listener.
        input_fn: Input function used by the default listener, injectable for tests.

    Returns:
        ``(range_min, range_max)`` mappings keyed by joint name.

    Raises:
        CalibrationError: If ``joints`` names an unknown joint, or (with
            ``require_motion``) a joint never moved.
        ConfigurationError: If ``poll_s`` is not positive.
    """
    if poll_s <= 0:
        raise ConfigurationError("poll_s must be positive")
    selected = tuple(joints) if joints is not None else model.joints
    for joint in selected:
        model.id_of(joint)  # validates
    ids = tuple(model.id_of(joint) for joint in selected)
    should_stop = _enter_listener(prompt, input_fn) if stop is None else stop
    mins: dict[str, int] = {}
    maxs: dict[str, int] = {}
    t0 = clock()
    while True:
        positions = bus.sync_read(ids, "present_position")
        for joint, mid in zip(selected, ids, strict=True):
            value = positions[mid]
            mins[joint] = min(mins.get(joint, value), value)
            maxs[joint] = max(maxs.get(joint, value), value)
        if should_stop():
            break
        if timeout_s is not None and clock() - t0 >= timeout_s:
            break
        sleep(poll_s)
    if require_motion:
        still = [joint for joint in selected if maxs[joint] - mins[joint] < min_span_ticks]
        if still:
            raise CalibrationError(
                f"No motion recorded for {still}; move every joint through its travel "
                "before finishing calibration"
            )
    return mins, maxs


def calibrate(
    bus: Bus,
    model: RobotModel,
    *,
    role: str = "follower",
    should_stop: Callable[[], bool] | None = None,
    full_turn: bool = False,
    prompt: Callable[[str], None] = print,
    input_fn: Callable[[str], str] = input,
    poll_s: float = 0.02,
) -> Calibration:
    """Run the full calibration flow and persist the result to the motors.

    Steps: torque off -> reset calibration -> half-turn homing -> record travel -> build,
    validate and write the calibration. Joints listed in :data:`FULL_TURN_JOINTS` are
    recorded over ``0..4095`` unless ``full_turn`` is true (LeRobot's current convention).

    Args:
        bus: Open transport.
        model: Arm model.
        role: ``"follower"`` or ``"leader"``; recorded in the prompts and the returned
            calibration is identical (the mechanics are the same).
        should_stop: Predicate that ends travel recording; defaults to "press Enter".
        full_turn: Use the recorded range for continuous-rotation joints instead of
            forcing ``0..4095``.
        prompt: Output function for instructions.
        input_fn: Input function for the default Enter listener.
        poll_s: Sampling period while recording.

    Returns:
        The calibration that was written to the motors.

    Raises:
        CalibrationError: If a joint did not move, or the recorded result is invalid.
        ConfigurationError: If ``role`` is unknown.
    """
    if role not in ("follower", "leader"):
        raise ConfigurationError(f"role must be 'follower' or 'leader', got {role!r}")
    bus.disable_torque(model.motor_ids)
    reset_calibration(bus, model)
    set_half_turn_homings(bus, model)
    prompt(
        f"Calibrating {model.name} ({role}): keep every joint loose, then move each one "
        "slowly through its full mechanical travel."
    )
    mins, maxs = record_ranges(
        bus,
        model,
        stop=should_stop,
        poll_s=poll_s,
        require_motion=False,
        prompt="Move every joint through its full travel, then press Enter... ",
        input_fn=input_fn,
    )
    if not full_turn:
        still = [
            joint
            for joint in model.joints
            if joint not in FULL_TURN_JOINTS and maxs[joint] - mins[joint] < 1
        ]
        if still:
            raise CalibrationError(
                f"No motion recorded for {still}; move every joint through its travel "
                "before finishing calibration"
            )
    offsets = {
        joint: cal.homing_offset for joint, cal in read_calibration(bus, model).joints.items()
    }
    joints: dict[str, JointCalibration] = {}
    for joint, mid in zip(model.joints, model.motor_ids, strict=True):
        lo, hi = mins[joint], maxs[joint]
        if not full_turn and joint in FULL_TURN_JOINTS:
            lo, hi = 0, RESOLUTION_MAX
        joints[joint] = JointCalibration(
            motor_id=mid,
            range_min=lo,
            range_max=hi,
            homing_offset=offsets[joint],
        )
    calibration = Calibration(joints)
    write_calibration(bus, model, calibration)
    return calibration
