"""Calibration data: homing offsets, joint ranges and the degrees <-> tick conversion.

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

import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

from pysoarm.constants import RESOLUTION_MAX
from pysoarm.exceptions import CalibrationError
from pysoarm.models.base import RobotModel

__all__ = ["HOMING_OFFSET_MAX", "Calibration", "JointCalibration"]

#: The reference arm uses 4095 as the full-scale raw value (``resolution - 1``).
_FULL_SCALE = float(RESOLUTION_MAX)
#: Largest homing offset the 11-bit sign-magnitude register can carry.
HOMING_OFFSET_MAX = (1 << 11) - 1


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
        if abs(self.homing_offset) > HOMING_OFFSET_MAX:
            raise CalibrationError(
                f"Motor {self.motor_id}: homing_offset {self.homing_offset} does not fit "
                f"the +/-{HOMING_OFFSET_MAX} register"
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
