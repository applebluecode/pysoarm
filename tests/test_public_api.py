"""The package must be importable, typed and free of import side effects."""

from __future__ import annotations

import importlib
from pathlib import Path

import pysoarm
from pysoarm.arm import ErrorPolicy, LoopStats, SOArm
from pysoarm.calibration import __all__ as calibration_all
from pysoarm.cli.main import EXIT_ERROR, EXIT_OK, EXIT_USAGE, build_parser, main


def test_version_is_a_string() -> None:
    assert isinstance(pysoarm.__version__, str)
    assert pysoarm.__version__


def test_package_ships_py_typed_marker() -> None:
    package_dir = Path(pysoarm.__file__).parent
    assert (package_dir / "py.typed").is_file()


def test_all_names_exist() -> None:
    for name in pysoarm.__all__:
        assert hasattr(pysoarm, name), f"{name} is exported but not defined"


def test_submodules_import_cleanly() -> None:
    for module in (
        "pysoarm.arm",
        "pysoarm.bus",
        "pysoarm.bus.base",
        "pysoarm.bus.registers",
        "pysoarm.calibration",
        "pysoarm.calibration.flow",
        "pysoarm.calibration.model",
        "pysoarm.cli",
        "pysoarm.cli.commands.bench",
        "pysoarm.cli.commands.bus",
        "pysoarm.cli.commands.calibration",
        "pysoarm.cli.commands.diagnostics",
        "pysoarm.cli.commands.kinematics",
        "pysoarm.cli.factories",
        "pysoarm.cli.metrics",
        "pysoarm.cli.parser",
        "pysoarm.constants",
        "pysoarm.diagnostics",
        "pysoarm.exceptions",
        "pysoarm.gripper",
        "pysoarm.kinematics",
        "pysoarm.loop",
        "pysoarm.models",
        "pysoarm.safety",
        "pysoarm.state",
        "pysoarm.trajectory",
        "pysoarm.tuning",
    ):
        importlib.import_module(module)


def test_legacy_module_paths_survive_structural_refactors() -> None:
    """Extraction is allowed only behind a stable import path."""
    assert SOArm.__name__ == "SOArm"
    assert LoopStats.__name__ == "LoopStats"
    assert ErrorPolicy is not None
    assert calibration_all == [
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
    assert (EXIT_OK, EXIT_ERROR, EXIT_USAGE) == (0, 1, 2)
    assert callable(build_parser) and callable(main)
