"""Shared pytest fixtures.

Hardware tests are opt-in: they need ``PYSOARM_HARDWARE_PORT`` to point at a real arm, and
they skip otherwise so that CI stays green on machines without servos.
"""

from __future__ import annotations

import json
import os
from collections.abc import Iterator
from pathlib import Path

import pytest

DATA_DIR = Path(__file__).parent / "data"


@pytest.fixture(scope="session")
def calibration_path() -> Path:
    """Path to a LeRobot-compatible calibration fixture (six STS3215 motors)."""
    return DATA_DIR / "so101_follower_calibration.json"


@pytest.fixture
def calibration_dict(calibration_path: Path) -> dict[str, dict[str, int]]:
    """The calibration fixture, parsed."""
    data: dict[str, dict[str, int]] = json.loads(calibration_path.read_text())
    return data


@pytest.fixture(scope="session")
def hardware_port() -> Iterator[str]:
    """Serial port of a real arm, or skip the test."""
    port = os.environ.get("PYSOARM_HARDWARE_PORT")
    if not port:
        pytest.skip("set PYSOARM_HARDWARE_PORT=/dev/ttyACM0 to run hardware tests")
    yield port
