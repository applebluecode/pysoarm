"""Every example must run offline against the fake bus (P0-4)."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).parents[1] / "examples"


@pytest.mark.parametrize(
    ("script", "extra"),
    [
        ("read_joints.py", []),
        ("hold_position.py", ["--seconds", "0.05"]),
        ("move_to.py", ["--duration", "0.05"]),
        ("control_loop.py", ["--seconds", "0.05"]),
        ("replay_trajectory.py", []),
        ("scan_bus.py", []),
    ],
)
def test_example_runs_against_the_fake_bus(script: str, extra: list[str]) -> None:
    result = subprocess.run(
        [sys.executable, str(EXAMPLES / script), "--fake", *extra],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip()


def test_no_example_still_raises_not_implemented() -> None:
    for path in EXAMPLES.glob("*.py"):
        assert "NotImplementedError" not in path.read_text(encoding="utf-8")
