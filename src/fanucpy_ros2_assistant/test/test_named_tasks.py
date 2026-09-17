# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Check file validation and human wording without ROS or hardware."""

import json
from pathlib import Path

import pytest

from fanucpy_ros2_assistant.named_tasks import (
    TaskCatalog,
    TaskFileError,
    decode_task,
    parse_task_request,
    validate_task,
)
from fanucpy_ros2_task_planner.prompt_parser import (
    PromptParseError,
    PromptParser,
)


TASKS = Path(__file__).resolve().parents[1] / "tasks"
NAMES = ("move_left_demo", "home_demo", "robot_status")


def task_data():
    """Read a bundled example to mutate only in isolated test memory."""
    return json.loads((TASKS / "move_left_demo.json").read_text())


@pytest.mark.parametrize("prompt", [
    "please run the move left example",
    "could you please run move_left_demo for me please?",
    "can u execute the file move_left_demo.json?",
    "start task move_left_demo",
    "/run move_left_demo",
])
def test_friendly_run_wording_has_one_exact_task(prompt):
    result = parse_task_request(prompt, NAMES)
    assert result.operation == "run"
    assert result.name == "move_left_demo"


@pytest.mark.parametrize("prompt", [
    "preview the move left example", "/preview move_left_demo.json",
])
def test_preview_is_not_a_run_request(prompt):
    assert parse_task_request(prompt, NAMES).operation == "preview"


@pytest.mark.parametrize("prompt", [
    "/tasks", "what examples can i run?", "show me the tasks",
])
def test_list_requests(prompt):
    assert parse_task_request(prompt, NAMES).operation == "list"


@pytest.mark.parametrize("prompt", [
    "run task move_lefft_demo", "run move_left_demo at 200 mm/s",
    "run move_left_demo then go home", "run home_demo or move_left_demo",
    "don't run move_left_demo", "never run the move left example",
    "run file ../../secret.json", "run file /tmp/move.py",
    "run move_left_demo; echo hello", "execute evil.sh",
    "run task move_left_demo twice", "/run unknown", "run the example",
    "run move_left_demo in tool frame", "run HOME_P",
    "launch move_left_demo", "call evil.py", "open file evil.sh",
    "do the move left example", "load task move_left_demo",
])
def test_unsafe_ambiguous_or_modified_task_is_not_forwarded_to_model(prompt):
    with pytest.raises(TaskFileError):
        parse_task_request(prompt, NAMES)


@pytest.mark.parametrize("prompt", [
    "go home", "drop it at 200 mm/s", "move left 5 mm", "what do you see",
    "run TP program HOME_P", "please execute program HOME_P",
])
def test_existing_command_paths_are_not_rewritten(prompt):
    assert parse_task_request(prompt, NAMES) is None


def test_alias_collision_requires_unambiguous_catalog():
    with pytest.raises(TaskFileError):
        parse_task_request("run move left example", (
            "move_left_demo", "move_left_example",
        ))


def test_bundled_examples_are_single_step_and_valid():
    catalog = TaskCatalog(str(TASKS))
    assert set(catalog.names) == set(NAMES)
    for name in catalog.names:
        task = catalog.get(name)
        assert len(task.sha256) == 64
        validate_task(task, PromptParser(), "fanuc_world")
    assert catalog.get("move_left_demo").plan.offset == (-5, 0, 0, 0, 0, 0)
    assert catalog.get("home_demo").plan.program_name == "HOME_P"
    assert catalog.get("robot_status").plan is None


@pytest.mark.parametrize("key, value", [
    ("delta_x_mm", float("nan")), ("delta_x_mm", float("inf")),
    ("delta_x_mm", "-5"), ("delta_x_mm", True),
    ("delta_x_mm", 10 ** 1000),
    ("velocity_mm_s", True), ("velocity_mm_s", 20.5),
    ("velocity_mm_s", -1), ("velocity_mm_s", 65536),
    ("schema_version", True), ("schema_version", 2),
    ("name", "../foo"), ("name", "different_name"),
    ("frame_id", ""), ("description", "hello\x1b[31m"),
    ("action", "shell"), ("command", "rm -rf anything"),
])
def test_invalid_values_and_unknown_fields_rejected(key, value):
    data = task_data()
    data[key] = value
    with pytest.raises(TaskFileError):
        decode_task(json.dumps(data).encode(), "move_left_demo", "test")


def test_missing_axis_rejected_instead_of_guessing():
    data = task_data()
    del data["delta_y_mm"]
    with pytest.raises(TaskFileError):
        decode_task(json.dumps(data).encode(), "move_left_demo", "test")


@pytest.mark.parametrize("raw", [
    b"{}", b"[]", b"{", b"{\"action\":1,\"action\":2}", b"\xff",
    b" " * 16385, b"[" * 2000,
])
def test_malformed_duplicate_or_large_files_rejected(raw):
    with pytest.raises(TaskFileError):
        decode_task(raw, "move_left_demo", "test")


def test_frames_and_jog_limits_still_apply():
    task = TaskCatalog(str(TASKS)).get("move_left_demo")
    with pytest.raises(TaskFileError, match="frame"):
        validate_task(task, PromptParser(), "tool")
    with pytest.raises(PromptParseError):
        validate_task(task, PromptParser(
            default_translation_mm=1, max_translation_step_mm=1,
        ), "fanuc_world")


def test_cartesian_target_requires_all_axes_with_mm_degree_fields():
    data = {
        "schema_version": 1, "name": "target", "description": "Test only",
        "action": "cartesian_target", "frame_id": "fanuc_world",
        "x_mm": 1, "y_mm": 2, "z_mm": 3,
        "w_deg": 4, "p_deg": 5, "r_deg": 6, "velocity_mm_s": 0,
    }
    task = decode_task(json.dumps(data).encode(), "target", "test")
    assert task.plan.target == (1, 2, 3, 4, 5, 6)
    validate_task(task, PromptParser(), "fanuc_world")


def test_task_snapshot_does_not_change_after_disk_edit(tmp_path):
    path = tmp_path / "move_left_demo.json"
    path.write_bytes((TASKS / path.name).read_bytes())
    catalog = TaskCatalog(str(tmp_path))
    first = catalog.get("move_left_demo")
    path.write_text("invalid replacement")
    assert catalog.get("move_left_demo") is first


def test_symlink_cannot_escape_configured_directory(tmp_path):
    outside = tmp_path / "outside.json"
    outside.write_text("{}")
    root = tmp_path / "tasks"
    root.mkdir()
    (root / "outside.json").symlink_to(outside)
    with pytest.raises(TaskFileError, match="inside"):
        TaskCatalog(str(root))


def test_builtin_catalog_accepts_colcon_symlink_install_layout(tmp_path):
    source = tmp_path / "source"
    installed = tmp_path / "install" / "tasks"
    source.mkdir()
    installed.mkdir(parents=True)
    task = source / "robot_status.json"
    task.write_text(
        '{"schema_version":1,"name":"robot_status",'
        '"description":"Read status.","action":"status"}',
        encoding="utf-8",
    )
    (installed / task.name).symlink_to(task)

    catalog = TaskCatalog(
        str(installed), allow_external_symlinks=True
    )

    assert catalog.names == ("robot_status",)
    assert catalog.get("robot_status").source == str(task.resolve())


def test_catalog_does_not_load_python_files(tmp_path):
    (tmp_path / "code.py").write_text("raise AssertionError('never run')")
    assert TaskCatalog(str(tmp_path)).names == ()
