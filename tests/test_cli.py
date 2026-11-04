"""CLI: argument wiring, exit codes and the environment doctor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

import pysoarm.cli.main as cli_main
from pysoarm.arm import SOArm
from pysoarm.bus.fake import FakeBus
from pysoarm.cli.main import EXIT_ERROR, EXIT_USAGE, build_parser, main


def test_parser_exposes_every_command() -> None:
    parser = build_parser()
    subparsers = [
        action for action in parser._actions if isinstance(action, argparse._SubParsersAction)
    ]
    assert len(subparsers) == 1
    assert set(subparsers[0].choices) == {"scan", "read", "monitor", "hold", "doctor"}


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
    monkeypatch.setattr(cli_main, "_make_arm", lambda args: SOArm(bus=bus))


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
    monkeypatch.setattr(cli_main, "_make_bus", lambda args: bus)
    assert main(["scan", "--port", "/dev/fake"]) == 0
    out = capsys.readouterr().out
    assert "1  777" in out
    assert "2  777" in out


def test_scan_json_output(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bus = FakeBus(motor_ids=(1,), model_number=777)
    monkeypatch.setattr(cli_main, "_make_bus", lambda args: bus)
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

    monkeypatch.setattr("pysoarm.cli.main.time.sleep", interrupt)
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

    monkeypatch.setattr("pysoarm.cli.main.time.sleep", interrupt)
    assert main(["monitor", "--port", "/dev/fake", "--hz", "10"]) == 0
    assert "shoulder_pan" in capsys.readouterr().out


def test_doctor_warns_about_modemmanager_without_failing(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(cli_main, "_service_active", lambda name: True)
    # /dev/null is held open by several daemons, so neutralise the holder scan here: this
    # test is about ModemManager being a warning, not about the holder check.
    monkeypatch.setattr(cli_main, "_port_holders", lambda path: [])
    code = main(["doctor", "--port", "/dev/null"])  # a character device that exists
    captured = capsys.readouterr().out
    assert code == 0
    assert "warn ModemManager" in captured


def test_version_string_includes_the_number(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as excinfo:
        main(["--version"])
    assert excinfo.value.code == 0
    assert capsys.readouterr().out.startswith("pysoarm ")
