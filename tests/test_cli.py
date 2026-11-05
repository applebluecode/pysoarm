"""CLI: argument wiring, exit codes and the environment doctor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from pysoarm.arm import SOArm
from pysoarm.bus.fake import FakeBus
from pysoarm.cli import factories
from pysoarm.cli.commands import diagnostics
from pysoarm.cli.commands import kinematics as kinematics_commands
from pysoarm.cli.main import EXIT_ERROR, EXIT_USAGE, build_parser, main
from pysoarm.exceptions import ConfigurationError
from pysoarm.kinematics import Pose


def test_parser_exposes_every_command() -> None:
    parser = build_parser()
    subparsers = [
        action for action in parser._actions if isinstance(action, argparse._SubParsersAction)
    ]
    assert len(subparsers) == 1
    assert set(subparsers[0].choices) == {
        "scan",
        "read",
        "monitor",
        "hold",
        "calibrate",
        "calibration",
        "bench",
        "doctor",
        "pose",
        "ik",
    }


def test_missing_command_is_a_usage_error() -> None:
    with pytest.raises(SystemExit) as excinfo:
        main([])
    assert excinfo.value.code == 2


def test_doctor_reports_a_missing_device(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["doctor", "--port", "/dev/pysoarm-does-not-exist"])
    captured = capsys.readouterr()
    assert code == EXIT_ERROR
    assert "device exists" in captured.out
    assert "FAIL" in captured.out


def test_doctor_fails_for_a_regular_file(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path = tmp_path / "not-a-tty"
    path.write_text("x", encoding="utf-8")
    code = main(["doctor", "--port", str(path)])
    captured = capsys.readouterr()
    assert code == EXIT_ERROR
    assert "is a character device" in captured.out


def test_hold_refuses_without_confirmation(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["hold", "--port", "/dev/null"])
    assert code == EXIT_USAGE
    assert "--yes" in capsys.readouterr().err


def test_scan_reports_an_unopenable_port(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["scan", "--port", "/dev/pysoarm-does-not-exist"])
    captured = capsys.readouterr()
    assert code == EXIT_ERROR
    assert "Cannot open" in captured.err


def test_monitor_validates_the_rate() -> None:
    assert main(["monitor", "--port", "/dev/null", "--hz", "0"]) == EXIT_USAGE


# --------------------------------------------------------------------------- #
# Happy paths, with the bus factory redirected to the in-memory fake.
# --------------------------------------------------------------------------- #
def fake_arm(monkeypatch: pytest.MonkeyPatch, bus: FakeBus) -> None:
    """Redirect the CLI's arm factory to an SOArm on the fake bus."""
    monkeypatch.setattr(factories, "make_arm", lambda args: SOArm(bus=bus))


def ready_bus() -> FakeBus:
    """A fake bus whose travel makes the default calibration usable."""
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


def test_scan_prints_ids_and_models(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus(motor_ids=(1, 2), model_number=777)
    monkeypatch.setattr(factories, "make_bus", lambda args: bus)
    assert main(["scan", "--port", "/dev/fake"]) == 0
    out = capsys.readouterr().out
    assert "1  777" in out
    assert "2  777" in out


def test_scan_json_output(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus(motor_ids=(1,), model_number=777)
    monkeypatch.setattr(factories, "make_bus", lambda args: bus)
    assert main(["scan", "--port", "/dev/fake", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload == {"port": "/dev/fake", "motors": {"1": 777}}


def test_read_prints_a_snapshot(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fake_arm(monkeypatch, ready_bus())
    assert main(["read", "--port", "/dev/fake"]) == 0
    out = capsys.readouterr().out
    assert "shoulder_pan=" in out
    assert "temp_c=" in out


def test_read_json_output(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fake_arm(monkeypatch, ready_bus())
    assert main(["read", "--port", "/dev/fake", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert len(payload["q_deg"]) == 6
    assert payload["joints"][0] == "shoulder_pan"


def test_hold_with_confirmation_runs_until_interrupted(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = ready_bus()
    fake_arm(monkeypatch, bus)

    def interrupt(_seconds: float) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr("pysoarm.cli.commands.bus.time.sleep", interrupt)
    assert main(["hold", "--port", "/dev/fake", "--yes"]) == 0
    out = capsys.readouterr().out
    assert "holding 6 joints" in out
    assert "torque disabled" in out
    assert bus.get_register(1, "torque_enable") == 0


def test_monitor_prints_a_header_then_interrupts(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    fake_arm(monkeypatch, ready_bus())

    def interrupt(_seconds: float) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr("pysoarm.cli.commands.bus.time.sleep", interrupt)
    assert main(["monitor", "--port", "/dev/fake", "--hz", "10"]) == 0
    assert "shoulder_pan" in capsys.readouterr().out


def test_doctor_warns_about_modemmanager_without_failing(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(diagnostics, "_service_active", lambda name: True)
    # /dev/null is held open by several daemons, so neutralise the holder scan here: this
    # test is about ModemManager being a warning, not about the holder check.
    monkeypatch.setattr(diagnostics, "_port_holders", lambda path: [])
    code = main(["doctor", "--port", "/dev/null"])  # a character device that exists
    captured = capsys.readouterr().out
    assert code == 0
    assert "warn ModemManager" in captured


def test_version_string_includes_the_number(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as excinfo:
        main(["--version"])
    assert excinfo.value.code == 0
    assert capsys.readouterr().out.startswith("pysoarm ")


# --------------------------------------------------------------------------- #
# Cartesian sub-commands (kinematics stubbed: no URDF in the test)
# --------------------------------------------------------------------------- #
class StubKinematics:
    """Fixed pose / solution, so the CLI wiring can be tested without a URDF."""

    def __init__(self) -> None:
        self.pose = Pose.from_xyz_rpy(0.2, 0.1, 0.05)
        self.forward_calls: list[tuple[float, ...]] = []

    @property
    def joint_names(self) -> tuple[str, ...]:
        return ("j1", "j2")

    def forward(self, q_deg: object) -> object:
        self.forward_calls.append(tuple(q_deg))  # type: ignore[arg-type]
        return self.pose

    def inverse(self, pose: object, **kwargs: object) -> tuple[float, ...]:
        del pose, kwargs
        return (10.0, 20.0)


def test_joint_list_parses_commas_and_spaces() -> None:
    assert kinematics_commands._joint_list("0, -30 60") == (0.0, -30.0, 60.0)
    with pytest.raises(ConfigurationError, match="Cannot parse joint values"):
        kinematics_commands._joint_list("0, oops")


def test_pose_with_explicit_joints(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    stub = StubKinematics()
    monkeypatch.setattr(factories, "make_kinematics", lambda args: stub)
    assert main(["pose", "--urdf", "arm.urdf", "--q", "0,0", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["joints_deg"] == [0.0, 0.0]
    assert payload["xyz_m"] == pytest.approx([0.2, 0.1, 0.05])
    assert stub.forward_calls == [(0.0, 0.0)]


def test_pose_human_readable_output(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_kinematics", lambda args: StubKinematics())
    assert main(["pose", "--urdf", "arm.urdf", "--q", "0,0"]) == 0
    out = capsys.readouterr().out
    assert "xyz_m" in out
    assert "rpy_deg" in out


def test_pose_needs_joints_or_a_port(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_kinematics", lambda args: StubKinematics())
    assert main(["pose", "--urdf", "arm.urdf"]) == EXIT_USAGE
    assert "--q or --port" in capsys.readouterr().err


def test_pose_can_read_the_arm(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    stub = StubKinematics()
    monkeypatch.setattr(factories, "make_kinematics", lambda args: stub)
    bus = ready_bus()
    monkeypatch.setattr(factories, "make_readonly_arm", lambda args: SOArm(bus=bus))
    assert main(["pose", "--urdf", "arm.urdf", "--port", "/dev/fake", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert len(payload["joints_deg"]) == 6
    assert payload["xyz_m"] == pytest.approx([0.2, 0.1, 0.05])


def test_ik_prints_the_solution(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_kinematics", lambda args: StubKinematics())
    assert main(["ik", "--urdf", "arm.urdf", "--xyz", "0.5", "0.5", "0.0", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["joints_deg"] == [10.0, 20.0]


def test_ik_accepts_orientation_and_seed(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_kinematics", lambda args: StubKinematics())
    args = [
        "ik",
        "--urdf",
        "arm.urdf",
        "--xyz",
        "0.5",
        "0.5",
        "0.0",
        "--rpy",
        "0",
        "0",
        "90",
        "--seed",
        "1,2",
    ]
    assert main(args) == 0
    assert capsys.readouterr().out.strip() == "10.00    20.00"


# --------------------------------------------------------------------------- #
# 0.1.1 commands: calibration inspection, bench and the stress doctor
# --------------------------------------------------------------------------- #
def test_calibration_show_json(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    monkeypatch.setattr(factories, "make_bus", lambda args: bus)
    assert main(["calibration", "show", "--port", "/dev/fake", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["shoulder_pan"]["range_min"] == 800
    assert len(payload) == 6


def test_calibration_export_writes_a_lerobot_file(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    monkeypatch.setattr(factories, "make_bus", lambda args: bus)
    output = tmp_path / "export.json"
    assert main(["calibration", "export", "--port", "/dev/fake", "--output", str(output)]) == 0
    assert json.loads(output.read_text())["gripper"]["range_max"] == 3200
    assert "wrote" in capsys.readouterr().out


def test_calibration_export_needs_an_output(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    monkeypatch.setattr(factories, "make_bus", lambda args: bus)
    assert main(["calibration", "export", "--port", "/dev/fake"]) == EXIT_USAGE
    assert "--output" in capsys.readouterr().err


def test_bench_reports_read_write_and_loop(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_bus", lambda args: ready_bus())
    args = [
        "bench",
        "--port",
        "/dev/fake",
        "--samples",
        "5",
        "--seconds",
        "0.01",
        "--hz",
        "200",
        "--json",
    ]
    assert main(args) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["backend"] == "fake"
    assert payload["read"]["mean_ms"] >= 0
    assert payload["loop"]["ticks"] >= 1


def test_bench_rejects_non_positive_arguments(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["bench", "--port", "/dev/fake", "--seconds", "0"]) == EXIT_USAGE
    assert "positive" in capsys.readouterr().err


def test_doctor_stress_reports_electrical_health(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(factories, "make_bus", lambda args: ready_bus())
    monkeypatch.setattr(diagnostics, "_service_active", lambda name: False)
    monkeypatch.setattr(diagnostics, "_port_holders", lambda path: [])
    code = main(["doctor", "--port", "/dev/null", "--stress", "0.01"])
    captured = capsys.readouterr().out
    assert code == 0
    assert "backend fake" in captured
    assert "no read timeouts" in captured
    assert "rustypot available" in captured
