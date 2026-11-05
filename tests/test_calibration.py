"""Calibration parsing, validation and the degrees <-> tick round trip."""

from __future__ import annotations

import json
from collections.abc import Callable
from pathlib import Path

import pytest

from pysoarm.calibration import Calibration, JointCalibration
from pysoarm.constants import RESOLUTION_MAX
from pysoarm.exceptions import CalibrationError, ConfigurationError
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


# --------------------------------------------------------------------------- #
# Public motor-side calibration flow (0.1.1)
# --------------------------------------------------------------------------- #
from pysoarm.bus.fake import FakeBus  # noqa: E402
from pysoarm.calibration import (  # noqa: E402
    FULL_TURN_JOINTS,
    calibrate,
    read_calibration,
    record_ranges,
    reset_calibration,
    set_half_turn_homings,
    write_calibration,
)


@pytest.fixture
def bus() -> FakeBus:
    """Six servos with a usable travel."""
    fake = FakeBus()
    for mid in fake.motor_ids:
        fake.set_register(mid, "min_position_limit", 800)
        fake.set_register(mid, "max_position_limit", 3200)
    fake.open()
    return fake


def test_read_calibration_reads_the_motor_registers(bus: FakeBus) -> None:
    for index, mid in enumerate(bus.motor_ids):
        bus.set_register(mid, "homing_offset", index + 1)
    calibration = read_calibration(bus, SO101)
    calibration.validate(SO101)
    assert [cal.motor_id for cal in calibration.joints.values()] == list(SO101.motor_ids)
    assert calibration.for_joint("shoulder_pan").homing_offset == 1
    assert calibration.for_joint("wrist_roll").range_max == 3200


def test_read_calibration_refuses_an_uncalibrated_arm() -> None:
    fake = FakeBus()
    for mid in fake.motor_ids:
        fake.set_register(mid, "min_position_limit", 2048)
        fake.set_register(mid, "max_position_limit", 2048)
    fake.open()
    with pytest.raises(CalibrationError, match="not calibrated"):
        read_calibration(fake, SO101)


def test_write_calibration_updates_every_register(bus: FakeBus, calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    write_calibration(bus, SO101, calibration)
    assert bus.get_register(1, "min_position_limit") == 732
    assert bus.get_register(6, "max_position_limit") == 2819
    assert bus.get_register(3, "homing_offset") == 1849


def test_reset_calibration_widens_the_travel(bus: FakeBus, calibration_path: Path) -> None:
    write_calibration(bus, SO101, Calibration.from_json(calibration_path))
    reset_calibration(bus, SO101)
    for mid in bus.motor_ids:
        assert bus.get_register(mid, "min_position_limit") == 0
        assert bus.get_register(mid, "max_position_limit") == RESOLUTION_MAX
        assert bus.get_register(mid, "homing_offset") == 0
        assert bus.get_register(mid, "torque_enable") == 0


def test_set_half_turn_homings_uses_the_feetech_convention(bus: FakeBus) -> None:
    bus.set_register(1, "present_position", 2048)
    bus.set_register(2, "present_position", 4095)  # would be 2048: not representable
    bus.set_register(3, "present_position", 0)  # would be -2047
    offsets = set_half_turn_homings(bus, SO101)
    assert offsets["shoulder_pan"] == 1
    assert offsets["shoulder_lift"] == 2047  # clamped to the 11-bit register
    assert offsets["elbow_flex"] == -2047
    assert bus.get_register(2, "homing_offset") == 2047


def _moving_stop(bus: FakeBus, positions: list[int]) -> Callable[[], bool]:
    """Stop predicate that moves every joint through ``positions`` over four samples."""
    counter = {"n": 0}

    def predicate() -> bool:
        index = min(counter["n"], len(positions) - 1)
        for mid in bus.motor_ids:
            bus.set_register(mid, "present_position", positions[index])
        counter["n"] += 1
        return counter["n"] > len(positions)

    return predicate


def test_record_ranges_tracks_min_and_max(bus: FakeBus) -> None:
    mins, maxs = record_ranges(
        bus,
        SO101,
        stop=_moving_stop(bus, [1000, 1200, 2000, 3000]),
        sleep=lambda _seconds: None,
    )
    assert mins == dict.fromkeys(SO101.joints, 1000)
    assert maxs == dict.fromkeys(SO101.joints, 3000)


def test_record_ranges_refuses_a_static_arm(bus: FakeBus) -> None:
    with pytest.raises(CalibrationError, match="No motion"):
        record_ranges(bus, SO101, stop=lambda: True, sleep=lambda _seconds: None)


def test_record_ranges_can_skip_the_motion_check(bus: FakeBus) -> None:
    mins, maxs = record_ranges(
        bus,
        SO101,
        stop=lambda: True,
        sleep=lambda _seconds: None,
        require_motion=False,
    )
    assert mins == maxs  # a snapshot is still a valid, if useless, range


def test_calibrate_runs_the_whole_flow(bus: FakeBus) -> None:
    calibration = calibrate(
        bus,
        SO101,
        role="follower",
        should_stop=_moving_stop(bus, [1000, 1200, 2000, 3000]),
        input_fn=lambda prompt: "",
        prompt=lambda message: None,
    )
    calibration.validate(SO101)
    for joint in SO101.joints:
        cal = calibration.for_joint(joint)
        if joint in FULL_TURN_JOINTS:
            assert (cal.range_min, cal.range_max) == (0, RESOLUTION_MAX)
        else:
            assert (cal.range_min, cal.range_max) == (1000, 3000)
    # written through to the motors
    assert bus.get_register(1, "min_position_limit") == 1000
    assert bus.get_register(6, "max_position_limit") == 3000
    # and readable back
    assert read_calibration(bus, SO101).to_dict() == calibration.to_dict()


def test_calibrate_full_turn_uses_the_recorded_range(bus: FakeBus) -> None:
    calibration = calibrate(
        bus,
        SO101,
        full_turn=True,
        should_stop=_moving_stop(bus, [1000, 1200, 2000, 3000]),
        input_fn=lambda prompt: "",
        prompt=lambda message: None,
    )
    assert calibration.for_joint("wrist_roll").range_min == 1000
    assert calibration.for_joint("wrist_roll").range_max == 3000


def test_calibrate_rejects_an_unknown_role(bus: FakeBus) -> None:
    with pytest.raises(ConfigurationError, match="role must be"):
        calibrate(bus, SO101, role="observer")


def test_calibrate_reports_a_static_arm(bus: FakeBus) -> None:
    with pytest.raises(CalibrationError, match="No motion"):
        calibrate(
            bus,
            SO101,
            should_stop=lambda: True,
            input_fn=lambda prompt: "",
            prompt=lambda message: None,
        )


def test_validate_rejects_duplicate_motor_ids(calibration_path: Path) -> None:
    calibration = Calibration.from_json(calibration_path)
    joints = dict(calibration.joints)
    joints["gripper"] = JointCalibration(motor_id=1, range_min=0, range_max=100)
    with pytest.raises(CalibrationError, match="assigned to both"):
        Calibration(joints).validate(SO101)
