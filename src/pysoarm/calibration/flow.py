"""Motor-side calibration flow: read, reset, record travel and persist.

This module changes when the *procedure* changes (how homing offsets are recorded, how travel
is sampled, how interactive input works). The data model and unit conversions live in
:mod:`pysoarm.calibration.model`; this module only orchestrates bus transactions around them.
"""

from __future__ import annotations

import contextlib
import threading
import time
from collections.abc import Callable, Sequence

from pysoarm.bus.base import Bus
from pysoarm.calibration.model import (
    HOMING_OFFSET_MAX,
    Calibration,
    JointCalibration,
)
from pysoarm.constants import RESOLUTION_MAX
from pysoarm.exceptions import CalibrationError, ConfigurationError
from pysoarm.models.base import RobotModel

__all__ = [
    "FULL_TURN_JOINTS",
    "calibrate",
    "read_calibration",
    "record_ranges",
    "reset_calibration",
    "set_half_turn_homings",
    "write_calibration",
]

#: Raw position the servo reports after a correct half-turn homing. The 11-bit
#: sign-magnitude register cannot represent 2048, so 2047 is the canonical value
#: (see the improvement report, P0-3).
_HALF_TURN_RAW = RESOLUTION_MAX // 2
#: Joints that rotate continuously and are calibrated over the whole encoder range.
FULL_TURN_JOINTS: tuple[str, ...] = ("wrist_roll",)


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
        offsets[joint] = min(HOMING_OFFSET_MAX, max(-HOMING_OFFSET_MAX, offset))
    bus.sync_write("homing_offset", {mid: offsets[model.joint_of(mid)] for mid in ids})
    return offsets


def _enter_listener(prompt: str, input_fn: Callable[[str], str] = input) -> Callable[[], bool]:
    """Return a non-blocking ``stop`` predicate that turns true after Enter.

    ``input`` runs on a daemon thread so polling continues while the operator moves the
    joints. This works on Linux, macOS and Windows without ``select``/``msvcrt``.
    """
    pressed = threading.Event()

    def listen() -> None:
        with contextlib.suppress(EOFError, OSError, KeyboardInterrupt):
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


def _wait_for_middle(
    wait_middle: Callable[[], None] | None,
    prompt: Callable[[str], None],
    input_fn: Callable[[str], str],
) -> None:
    """Block until the operator has placed every joint at mid-travel.

    ``wait_middle`` is injected by callers that own the interaction (a visual console, a
    headless test, a robot-holding fixture). Without it the library falls back to a plain
    Enter prompt; ``EOFError``/``OSError``/``KeyboardInterrupt`` are treated as "already in
    place" so a piped/non-interactive run still makes progress.
    """
    instruction = (
        "Move every joint to the middle of its travel (raw position near 2048), "
        "then press Enter... "
    )
    if wait_middle is not None:
        prompt(instruction)
        wait_middle()
        return
    with contextlib.suppress(EOFError, OSError, KeyboardInterrupt):
        input_fn(instruction)


def calibrate(
    bus: Bus,
    model: RobotModel,
    *,
    role: str = "follower",
    middle: bool = True,
    wait_middle: Callable[[], None] | None = None,
    should_stop: Callable[[], bool] | None = None,
    full_turn: bool = False,
    prompt: Callable[[str], None] = print,
    input_fn: Callable[[str], str] = input,
    poll_s: float = 0.02,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> Calibration:
    """Run the full calibration flow and persist the result to the motors.

    Steps: torque off -> reset calibration -> *move to mid-travel* -> half-turn homing ->
    record travel -> build, validate and write the calibration. Joints listed in
    :data:`FULL_TURN_JOINTS` are recorded over ``0..4095`` unless ``full_turn`` is true
    (LeRobot's current convention).

    The middle stage is what makes ``2048`` mean the mechanical middle, matching LeRobot's
    SO calibration. It runs **before** :func:`set_half_turn_homings`, so the offsets are
    computed from the pose the operator holds at that moment rather than from whatever pose
    the arm happened to be in. ``middle=False`` reproduces the pre-0.1.3 behaviour (homing at
    the current pose); the two conventions must not be mixed between arms that are later
    teleoperated together.

    Args:
        bus: Open transport.
        model: Arm model.
        role: ``"follower"`` or ``"leader"``; recorded in the prompts and the returned
            calibration is identical (the mechanics are the same).
        middle: Wait for the operator to place every joint at mid-travel before homing.
            Defaults to ``True`` (LeRobot parity).
        wait_middle: Injected blocking call that returns once the arm is at mid-travel. It
            must not print or read stdin. When omitted, ``input_fn`` is used.
        should_stop: Predicate that ends travel recording; defaults to "press Enter".
        full_turn: Use the recorded range for continuous-rotation joints instead of
            forcing ``0..4095``.
        prompt: Output function for instructions.
        input_fn: Input function for the default Enter prompts.
        poll_s: Sampling period while recording.
        sleep: Sleep function used while recording, injectable for tests.
        clock: Monotonic clock used for the recording timeout, injectable for tests.

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
    if middle:
        _wait_for_middle(wait_middle, prompt, input_fn)
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
        sleep=sleep,
        clock=clock,
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
