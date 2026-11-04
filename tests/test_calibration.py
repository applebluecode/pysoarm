"""Calibration parsing, validation and the degrees <-> tick round trip."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from pysoarm.calibration import Calibration, JointCalibration
from pysoarm.constants import RESOLUTION_MAX
from pysoarm.exceptions import CalibrationError
from pysoarm.models import SO101


def test_fixture_loads_six_joints(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    assert list(calibration.joints) == list(SO101.joints)
    calibration.validate(SO101)
    assert calibration.for_joint("gripper").motor_id == 6


def test_range_endpoints_map_symmetrically(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    cal = calibration.for_joint("elbow_flex")
    low, high = cal.raw_to_deg(cal.range_min), cal.raw_to_deg(cal.range_max)
    assert low == pytest.approx(-high)
    assert high - low == pytest.approx(cal.span_deg)


def test_deg_to_raw_round_trips_within_one_tick(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    cal = calibration.for_joint("shoulder_pan")
    for raw in range(cal.range_min, cal.range_max + 1, 37):
        assert abs(cal.deg_to_raw(cal.raw_to_deg(raw)) - raw) <= 1


def test_deg_to_raw_clamps_to_the_encodable_range() -> None:
    cal = JointCalibration(motor_id=1, range_min=100, range_max=4000)
    assert cal.deg_to_raw(10_000.0) == RESOLUTION_MAX
    assert cal.deg_to_raw(-10_000.0) == 0


def test_clamp_deg_keeps_the_joint_inside_its_travel() -> None:
    cal = JointCalibration(motor_id=1, range_min=1000, range_max=3000)
    assert cal.clamp_deg(1e6) == pytest.approx(cal.raw_to_deg(3000))
    assert cal.clamp_deg(-1e6) == pytest.approx(cal.raw_to_deg(1000))
    assert cal.clamp_deg(0.0) == pytest.approx(0.0)


def test_drive_mode_flips_the_sign() -> None:
    normal = JointCalibration(motor_id=1, range_min=1000, range_max=3000, drive_mode=0)
    mirrored = JointCalibration(motor_id=1, range_min=1000, range_max=3000, drive_mode=1)
    raw = 2500
    assert mirrored.raw_to_deg(raw) == pytest.approx(-normal.raw_to_deg(raw))
    assert mirrored.deg_to_raw(mirrored.raw_to_deg(raw)) == pytest.approx(raw, abs=1)


def test_json_round_trip(calibration_path: Path, tmp_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    written = calibration.to_json(tmp_path / "out.json")
    assert Calibration.from_json(written).to_dict() == calibration.to_dict()
    assert json.loads(written.read_text())["shoulder_pan"]["id"] == 1


def test_validate_rejects_id_mismatch(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    joints = dict(calibration.joints)
    joints["shoulder_pan"] = JointCalibration(motor_id=6, range_min=0, range_max=100)
    with pytest.raises(CalibrationError, match="calibration is for motor 6"):
        Calibration(joints).validate(SO101)


def test_validate_rejects_missing_and_extra_joints() -> None:
    with pytest.raises(CalibrationError, match="missing="):
        Calibration({"shoulder_pan": JointCalibration(1, 0, 100)}).validate(SO101)
    with pytest.raises(CalibrationError, match="unexpected="):
        Calibration(
            {
                **{j: JointCalibration(i + 1, 0, 100) for i, j in enumerate(SO101.joints)},
                "extra": JointCalibration(7, 0, 100),
            }
        ).validate(SO101)


@pytest.mark.parametrize(
    ("data", "match"),
    [
        ({"shoulder_pan": {"range_max": 100}}, "Missing 'range_min'"),
        ({"shoulder_pan": {"range_min": 5, "range_max": 5}}, "range_min < range_max"),
        ({"shoulder_pan": {"range_min": 0, "range_max": 5000}}, "range_min < range_max"),
        ({"shoulder_pan": {"range_min": "abc", "range_max": 100}}, "not an integer"),
        ({}, "Calibration file is empty"),
    ],
)
def test_malformed_calibration_is_rejected(data: dict[str, object], match: str) -> None:
    with pytest.raises(CalibrationError, match=match):
        Calibration.from_dict(data)  # type: ignore[arg-type]


def test_missing_file_is_reported(tmp_path: Path) -> None:
    with pytest.raises(CalibrationError, match="not found"):
        Calibration.from_json(tmp_path / "nope.json")


def test_invalid_json_is_reported(tmp_path: Path) -> None:
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(CalibrationError, match="not valid JSON"):
        Calibration.from_json(path)
