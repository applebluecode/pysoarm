"""The package must be importable, typed and free of import side effects."""

from __future__ import annotations

import importlib
from pathlib import Path

import pysoarm


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
        "pysoarm.cli",
        "pysoarm.constants",
        "pysoarm.exceptions",
        "pysoarm.kinematics",
        "pysoarm.models",
        "pysoarm.safety",
        "pysoarm.state",
        "pysoarm.trajectory",
    ):
        importlib.import_module(module)
