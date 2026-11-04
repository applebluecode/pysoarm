"""Arm models: ids, aliases and consistency checks."""

from __future__ import annotations

import pytest

from pysoarm.exceptions import ConfigurationError
from pysoarm.models import MODELS, SO100, SO101, RobotModel, get_model


def test_so101_layout() -> None:
    assert SO101.joint_count == 6
    assert SO101.joints[0] == "shoulder_pan"
    assert SO101.gripper_joint == "gripper"
    assert SO101.gripper_id == 6
    assert SO101.id_of("elbow_flex") == 3
    assert SO101.joint_of(3) == "elbow_flex"


def test_aliases_resolve_case_insensitively() -> None:
    assert get_model("SO101") is SO101
    assert get_model("so101_follower") is SO101
    assert get_model("so100") is SO100
    assert MODELS["so_follower"] is SO101


def test_unknown_model_and_joint_are_configuration_errors() -> None:
    with pytest.raises(ConfigurationError, match="Unknown model"):
        get_model("franka")
    with pytest.raises(ConfigurationError, match="Unknown joint"):
        SO101.id_of("tail")
    with pytest.raises(ConfigurationError, match="Unknown motor id"):
        SO101.joint_of(42)


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"joints": (), "motor_ids": ()}, "no joints"),
        ({"joints": ("a", "b"), "motor_ids": (1,)}, "2 joints but 1 ids"),
        ({"joints": ("a", "a"), "motor_ids": (1, 2)}, "duplicate joint"),
        ({"joints": ("a", "b"), "motor_ids": (1, 1)}, "duplicate motor id"),
        (
            {"joints": ("a",), "motor_ids": (1,), "gripper_joint": "b"},
            "is not a joint",
        ),
    ],
)
def test_inconsistent_models_are_rejected(kwargs: dict[str, object], match: str) -> None:
    with pytest.raises(ConfigurationError, match=match):
        RobotModel(name="broken", **kwargs)  # type: ignore[arg-type]


def test_as_dict_is_json_friendly() -> None:
    payload = SO101.as_dict()
    assert payload["joints"] == list(SO101.joints)
    assert payload["motor_ids"] == [1, 2, 3, 4, 5, 6]
