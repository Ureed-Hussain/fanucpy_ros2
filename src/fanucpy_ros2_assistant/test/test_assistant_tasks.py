# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Prove support/task routing never needs a model or unguarded robot call."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fanucpy_ros2_assistant import assistant
from fanucpy_ros2_assistant.example_programs import ExampleCatalog
from fanucpy_ros2_assistant.named_tasks import TaskCatalog
from fanucpy_ros2_assistant.package_help import PackageConversation
from fanucpy_ros2_task_planner.prompt_parser import PromptParser


@pytest.fixture
def node(monkeypatch):
    """Supply a task catalog and prohibit accidental external operations."""
    unexpected = Mock(side_effect=AssertionError("Unexpected dispatch"))
    for name in ("process_prompt", "_execute_decision",
                 "_print_combined_status"):
        monkeypatch.setattr(assistant, name, unexpected)
    return SimpleNamespace(
        task_catalog=TaskCatalog(str(
            Path(__file__).resolve().parents[1] / "tasks"
        )),
        example_catalog=ExampleCatalog(str(
            Path(__file__).resolve().parents[1] / "config" / "examples.json"
        )),
        allowed_tasks=frozenset(),
        allowed_examples=frozenset(),
        package_conversation=PackageConversation(),
        frame_id="fanuc_world", parser=PromptParser(),
        latest_status=None,
        robot_context=Mock(return_value=("", None, None)),
        wait_for_vision=unexpected,
    )


@pytest.mark.parametrize("prompt", [
    "why is vision unavailable?", "why does go home fail?",
    "/ask move left 5 mm", "hello", "yes", "do it", "run it", "stop",
    "/tasks", "/examples", "what examples can i run?",
])
def test_help_and_list_do_not_reach_model_feedback_or_actions(node, prompt):
    assert assistant.process_assistant_prompt(node, prompt, True) == 0
    node.robot_context.assert_not_called()


@pytest.mark.parametrize("execute, prompt", [
    (False, "run task move_left_demo"),
    (True, "preview the move left example"),
    (False, "run home example"),
])
def test_preview_and_dry_run_never_dispatch(node, capsys, execute, prompt):
    assert assistant.process_assistant_prompt(node, prompt, execute) == 0
    assert "DRY RUN" in capsys.readouterr().out


def test_execution_requires_explicit_task_allowlist(node, capsys):
    assert assistant.process_assistant_prompt(
        node, "run move_left_demo", True
    ) == 1
    assert "allowed_tasks" in capsys.readouterr().out


@pytest.mark.parametrize("prompt", [
    "run teleop example", "run movite2 example",
])
def test_program_example_requires_its_own_allowlist(node, capsys, prompt):
    assert assistant.process_assistant_prompt(node, prompt, True) == 2
    assert "allowed_examples" in capsys.readouterr().out
    node.robot_context.assert_not_called()


@pytest.mark.parametrize("prompt", [
    "preview teleop example", "preview moveit2 example",
])
def test_program_example_preview_starts_nothing(node, monkeypatch, capsys, prompt):
    start = Mock(side_effect=AssertionError("Must not start"))
    monkeypatch.setattr(assistant, "run_foreground", start)
    assert assistant.process_assistant_prompt(node, prompt, True) == 0
    assert "DRY RUN" in capsys.readouterr().out
    start.assert_not_called()


def test_confirmed_program_uses_preflight_and_fixed_launcher(node, monkeypatch):
    node.allowed_examples = {"teleop"}
    preflight = Mock()
    launch = Mock(return_value=0)
    monkeypatch.setattr(assistant, "_preflight_example", preflight)
    monkeypatch.setattr(assistant, "_confirmation", Mock(return_value=True))
    monkeypatch.setattr(assistant, "run_foreground", launch)
    assert assistant.process_assistant_prompt(
        node, "please run the teleop example", True
    ) == 0
    assert preflight.call_count == 2
    program = launch.call_args.args[0]
    assert program.argv == (
        "ros2", "run", "fanucpy_ros2_examples",
        "fanucpy_keyboard_teleop",
    )


@pytest.mark.parametrize("prompt", [
    "don't run teleop example", "run moveit2 mode:=mock",
    "run teleop then moveit2",
])
def test_bad_program_requests_do_not_reach_model_or_process(node, monkeypatch,
                                                            prompt):
    start = Mock(side_effect=AssertionError("Must not start"))
    monkeypatch.setattr(assistant, "run_foreground", start)
    assert assistant.process_assistant_prompt(node, prompt, True) == 2
    start.assert_not_called()
    node.robot_context.assert_not_called()


def test_allowlisted_task_reuses_existing_guarded_executor(node, monkeypatch):
    node.allowed_tasks = {"move_left_demo"}
    execute = Mock(return_value=True)
    monkeypatch.setattr(assistant, "_execute_decision", execute)
    assert assistant.process_assistant_prompt(
        node, "please run the move left example", True
    ) == 0
    execute.assert_called_once()
    plan = execute.call_args.args[1]
    assert plan.delta_x_mm == -5.0
    assert plan.velocity_mm_s == 20


def test_failure_does_not_retry(node, monkeypatch, capsys):
    node.allowed_tasks = {"move_left_demo"}
    execute = Mock(return_value=False)
    monkeypatch.setattr(assistant, "_execute_decision", execute)
    assert assistant.process_assistant_prompt(
        node, "run move_left_demo", True
    ) == 1
    execute.assert_called_once()
    assert "will not retry" in capsys.readouterr().out


@pytest.mark.parametrize("prompt", [
    "don't run move_left_demo", "run unknown_task.py",
    "run move_left_demo at 200 mm/s", "run move_left_demo then go home",
])
def test_bad_requests_never_reach_robot_or_model(node, prompt):
    assert assistant.process_assistant_prompt(node, prompt, True) == 2
    node.robot_context.assert_not_called()


def test_status_task_is_read_only_without_motion_permission(node, monkeypatch):
    read = Mock()
    monkeypatch.setattr(assistant, "_print_combined_status", read)
    assert assistant.process_assistant_prompt(
        node, "run robot_status", False
    ) == 0
    read.assert_called_once_with(node)


def test_mismatched_frame_blocks_before_feedback_or_action(node):
    node.frame_id = "tool"
    node.allowed_tasks = {"move_left_demo"}
    assert assistant.process_assistant_prompt(
        node, "run move_left_demo", True
    ) == 2
    node.robot_context.assert_not_called()


def test_real_jog_handler_honors_driver_gate(node, monkeypatch):
    from fanucpy_ros2_task_planner import ollama_prompt

    node.allowed_tasks = {"move_left_demo"}
    node.check_execution_ready = Mock(side_effect=RuntimeError("disabled"))
    node.get_logger = Mock(return_value=Mock())
    node.execute = Mock(side_effect=AssertionError("Must not send"))
    monkeypatch.setattr(assistant, "_execute_decision",
                        ollama_prompt._execute_decision)
    assert assistant.process_assistant_prompt(
        node, "run move_left_demo", True
    ) == 1
    node.execute.assert_not_called()


def test_real_jog_handler_requires_confirmation(node, monkeypatch):
    from fanucpy_ros2_task_planner import ollama_prompt
    from fanucpy_ros2_task_planner.conversation import CartesianSnapshot

    node.allowed_tasks = {"move_left_demo"}
    node.check_execution_ready = Mock()
    node.robot_context_timeout_sec = 1
    node.wait_for_cartesian_state = Mock(return_value=object())
    node.cartesian_snapshot = Mock(return_value=CartesianSnapshot(
        "fanuc_world", 0, 0, 0, 0, 0, 0,
    ))
    node.execute = Mock(side_effect=AssertionError("Must not send"))
    confirmation = Mock(return_value=False)
    monkeypatch.setattr(ollama_prompt, "_confirm_execution", confirmation)
    monkeypatch.setattr(assistant, "_execute_decision",
                        ollama_prompt._execute_decision)
    assert assistant.process_assistant_prompt(
        node, "run move_left_demo", True
    ) == 1
    confirmation.assert_called_once()
    node.execute.assert_not_called()
