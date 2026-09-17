# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Exercise dispatch without starting ROS, contacting Ollama, or moving hardware."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fanucpy_ros2_assistant import assistant
from fanucpy_ros2_assistant.vision_motion import VisionMotionError
from fanucpy_ros2_interfaces.msg import DriverStatus


@pytest.fixture
def node():
    """Provide only local workflow state; no real node or action clients."""
    fake = SimpleNamespace(
        frame_id="fanuc_world",
        drop_target_mm_deg=(237.0, 720.0, -167.0, -179.0, 60.0, -175.0),
        drop_velocity_mm_s=25,
        enable_drop_target_motion=False,
        enable_home_program_call=False,
        home_program_name="HOME_P",
        robot_context_timeout_sec=1.0,
        latest_status=SimpleNamespace(
            state=DriverStatus.CONNECTED, max_cartesian_velocity_mm_s=325
        ),
        parser=SimpleNamespace(max_cartesian_velocity_mm_s=2000),
        wait_for_cartesian_state=Mock(return_value=object()),
        check_absolute_cartesian_execution_ready=Mock(),
        check_program_execution_ready=Mock(side_effect=lambda plan: plan),
        execute_tp_program=Mock(return_value=True),
    )
    fake.maximum_cartesian_velocity = (
        lambda: assistant.FanucpyAssistant.maximum_cartesian_velocity(fake)
    )
    fake.workflow_velocity_limit = (
        lambda prompt: assistant.FanucpyAssistant.workflow_velocity_limit(
            fake, prompt
        )
    )
    return fake


@pytest.fixture(autouse=True)
def no_external_dispatch(monkeypatch):
    """Fail if a workflow falls through to Ollama or an unmocked action."""
    monkeypatch.setattr(
        assistant, "process_prompt",
        Mock(side_effect=AssertionError("Unexpected Ollama dispatch")),
    )
    monkeypatch.setattr(
        assistant, "_execute_cartesian_stage",
        Mock(side_effect=AssertionError("Unexpected robot action")),
    )
    monkeypatch.setattr(
        assistant, "_confirmation",
        Mock(side_effect=AssertionError("Unexpected confirmation")),
    )


@pytest.mark.parametrize(
    "prompt, expected_speed",
    [
        ("drop it at 200 mm/s", 200),
        ("go to drop position at max speed", 325),
        ("could you drop it at full speed please?", 325),
        ("drop it", 25),
    ],
)
def test_dry_run_drop_uses_saved_pose_and_requested_speed(
    node, capsys, prompt, expected_speed
):
    assert assistant.process_assistant_prompt(node, prompt, False) == 0
    output = capsys.readouterr().out
    assert f"{expected_speed} mm/s" in output
    assert "[237.000, 720.000, -167.000, -179.000, 60.000, -175.000]" in output
    assert "DRY RUN" in output
    assert "does not release an object" in output
    node.check_absolute_cartesian_execution_ready.assert_not_called()


def test_drop_gate_stays_disabled_by_default(node, capsys):
    assert assistant.process_assistant_prompt(
        node, "drop it at 200 mm/s", True
    ) == 1
    assert "-p enable_drop_target_motion:=true" in capsys.readouterr().out
    node.check_absolute_cartesian_execution_ready.assert_not_called()


def test_declining_confirmation_does_not_send_drop(node, monkeypatch):
    node.enable_drop_target_motion = True
    confirmation = Mock(return_value=False)
    monkeypatch.setattr(assistant, "_confirmation", confirmation)
    assert assistant.process_assistant_prompt(node, "drop it", True) == 1
    assert confirmation.call_args.args[0] == "MOVE TO DROP"


@pytest.mark.parametrize(
    "prompt, expected_speed",
    [("drop it at 200 mm/s", 200), ("drop it at max speed", 325)],
)
def test_confirmed_drop_passes_exact_speed_to_guarded_stage(
    node, monkeypatch, prompt, expected_speed
):
    node.enable_drop_target_motion = True
    monkeypatch.setattr(assistant, "_confirmation", Mock(return_value=True))
    execute = Mock(return_value=True)
    monkeypatch.setattr(assistant, "_execute_cartesian_stage", execute)
    assert assistant.process_assistant_prompt(node, prompt, True) == 0
    execute.assert_called_once()
    assert execute.call_args.args[2].target == node.drop_target_mm_deg
    assert execute.call_args.args[2].velocity_mm_s == expected_speed


def test_driver_gate_failure_does_not_reach_confirmation(node):
    node.enable_drop_target_motion = True
    node.check_absolute_cartesian_execution_ready.side_effect = RuntimeError(
        "Motion commands are disabled"
    )
    assert assistant.process_assistant_prompt(node, "drop it", True) == 1


@pytest.mark.parametrize(
    "prompt",
    [
        "don't drop it",
        "drop it then go home",
        "drop it at the battery",
        "drop it at 326 mm/s",
        "drop it at -200 mm/s",
        "drop it at 200 mm/s or max speed",
        "drop it slowly",
        "go home at 200 mm/s",
        "go home at max speed",
    ],
)
def test_invalid_request_never_falls_through_to_model(node, prompt):
    node.enable_drop_target_motion = True
    node.enable_home_program_call = True
    assert assistant.process_assistant_prompt(node, prompt, True) == 2
    node.execute_tp_program.assert_not_called()
    node.check_absolute_cartesian_execution_ready.assert_not_called()


@pytest.mark.parametrize(
    "prompt", ["where is the drop position?", "describe the home program"]
)
def test_workflow_questions_only_preview_even_in_execute_mode(
    node, capsys, prompt
):
    node.enable_drop_target_motion = True
    node.enable_home_program_call = True
    assert assistant.process_assistant_prompt(node, prompt, True) == 0
    assert "DRY RUN" in capsys.readouterr().out
    node.execute_tp_program.assert_not_called()
    node.check_absolute_cartesian_execution_ready.assert_not_called()


def test_home_requires_its_own_gate(node, capsys):
    assert assistant.process_assistant_prompt(node, "go home", True) == 1
    assert "-p enable_home_program_call:=true" in capsys.readouterr().out
    node.execute_tp_program.assert_not_called()


def test_home_uses_configured_allowlisted_program(node, monkeypatch):
    node.enable_home_program_call = True
    node.home_program_name = "REVIEWED_HOME"
    confirmation = Mock(return_value=True)
    monkeypatch.setattr(assistant, "_confirmation", confirmation)
    assert assistant.process_assistant_prompt(node, "go home", True) == 0
    assert node.check_program_execution_ready.call_count == 2
    assert confirmation.call_args.args[0] == "RUN REVIEWED_HOME"
    node.execute_tp_program.assert_called_once()
    assert node.execute_tp_program.call_args.args[0].program_name == (
        "REVIEWED_HOME"
    )


def test_max_speed_requires_a_new_cartesian_sample(node):
    assert node.workflow_velocity_limit("drop it at max speed") == 325
    arguments = node.wait_for_cartesian_state.call_args.kwargs
    assert arguments["require_fresh"] is True
    assert arguments["received_after"] > 0


@pytest.mark.parametrize("missing", ["status", "state", "connection", "limit"])
def test_max_speed_never_uses_fallback_without_live_driver(node, missing):
    if missing == "status":
        node.latest_status = None
    elif missing == "state":
        node.wait_for_cartesian_state.return_value = None
    elif missing == "connection":
        node.latest_status.state = DriverStatus.DISCONNECTED
    else:
        node.latest_status.max_cartesian_velocity_mm_s = 0
    with pytest.raises(VisionMotionError, match="fresh robot feedback"):
        node.workflow_velocity_limit("drop it at max speed")
