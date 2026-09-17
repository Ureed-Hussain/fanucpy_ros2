# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Verify reviewed ROS example lookup without starting a process."""

import json
from pathlib import Path

import pytest

from fanucpy_ros2_assistant.example_programs import (
    check_conflicts,
    ExampleCatalog,
    ExampleError,
    parse_example_request,
)


REGISTRY = Path(__file__).resolve().parents[1] / "config" / "examples.json"


@pytest.fixture
def catalog():
    """Load the installed-format registry from the source tree."""
    return ExampleCatalog(str(REGISTRY))


def test_reviewed_commands_are_fixed_and_reuse_the_driver(catalog):
    assert catalog.names == ("teleop", "moveit2")
    assert catalog.get("teleop").argv == (
        "ros2", "run", "fanucpy_ros2_examples",
        "fanucpy_keyboard_teleop",
    )
    assert catalog.get("moveit2").argv == (
        "ros2", "launch", "fanuc_m10ia_moveit_config",
        "fanuc_m10ia_moveit_existing_driver.launch.py",
    )
    assert catalog.get("moveit2").requires_driver is True
    assert catalog.get("moveit2").requires_joint_states is True


@pytest.mark.parametrize("prompt, expected", [
    ("run teleop example", "teleop"),
    ("please start the keyboard teleop for me", "teleop"),
    ("could you execute teleoperation example please?", "teleop"),
    ("run moveit2 example", "moveit2"),
    ("run movite2 example", "moveit2"),
    ("launch move it 2", "moveit2"),
    ("open the moveit demo", "moveit2"),
])
def test_rough_exact_aliases_resolve_without_a_model(catalog, prompt, expected):
    request = parse_example_request(prompt, catalog)
    assert request.operation == "run"
    assert request.name == expected


@pytest.mark.parametrize("prompt", [
    "/examples", "what examples can I run?", "show me the examples",
])
def test_example_list_requests(catalog, prompt):
    assert parse_example_request(prompt, catalog).operation == "list"


@pytest.mark.parametrize("prompt", [
    "preview teleop example", "/preview movite2",
])
def test_preview_never_becomes_run(catalog, prompt):
    assert parse_example_request(prompt, catalog).operation == "preview"


@pytest.mark.parametrize("prompt", [
    "don't run teleop example", "never launch moveit2",
    "run teleop and moveit2", "run moveit2 mode:=mock",
    "run teleop twice", "run teleop in background",
    "run movite2 then execute home", "run /tmp/teleop.py",
])
def test_negated_modified_or_chained_example_is_rejected(catalog, prompt):
    with pytest.raises(ExampleError):
        parse_example_request(prompt, catalog)


@pytest.mark.parametrize("prompt", [
    "move left 5 mm", "run task move_left_demo",
    "run TP program HOME_P", "what do you see?",
])
def test_other_intents_are_not_claimed(catalog, prompt):
    assert parse_example_request(prompt, catalog) is None


def test_known_node_conflicts_are_rejected(catalog):
    with pytest.raises(ExampleError, match="move_group"):
        check_conflicts(catalog.get("moveit2"), ["move_group"])
    check_conflicts(catalog.get("moveit2"), ["fanucpy_driver"])


@pytest.mark.parametrize("mutation", [
    lambda data: data.update(schema_version=2),
    lambda data: data["examples"][0].update(kind="shell"),
    lambda data: data["examples"][0].update(target="../../script.py"),
    lambda data: data["examples"][0].update(arguments="--unsafe"),
    lambda data: data["examples"][0].update(requires_driver="true"),
    lambda data: data["examples"][0].update(extra="unexpected"),
    lambda data: data["examples"].append(data["examples"][0].copy()),
])
def test_malformed_or_unreviewable_registry_is_rejected(tmp_path, mutation):
    data = json.loads(REGISTRY.read_text())
    mutation(data)
    path = tmp_path / "examples.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ExampleError):
        ExampleCatalog(str(path))


def test_registry_is_snapshotted_until_restart(tmp_path):
    path = tmp_path / "examples.json"
    path.write_bytes(REGISTRY.read_bytes())
    catalog = ExampleCatalog(str(path))
    path.write_text("invalid replacement")
    assert catalog.get("teleop").target == "fanucpy_keyboard_teleop"
