# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import json
from urllib.error import URLError

import pytest

from fanucpy_ros2_task_planner.ollama_client import (
    DECISION_SCHEMA,
    OllamaPlanner,
    OllamaPlannerError,
    parse_model_decision,
)
from fanucpy_ros2_task_planner.prompt_parser import PromptParser
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    JointTargetPlan,
    TpProgramPlan,
)


def _decision(**overrides):
    data = {
        "message": "I interpreted a relative Cartesian movement.",
        "needs_clarification": False,
        "clarification_question": "",
        "action": "cartesian_jog",
        "delta_x_mm": -50.0,
        "delta_y_mm": 0.0,
        "delta_z_mm": 0.0,
        "delta_w_deg": 0.0,
        "delta_p_deg": 0.0,
        "delta_r_deg": 0.0,
        "velocity_mm_s": 200,
        "target_x_mm": 0.0,
        "target_y_mm": 0.0,
        "target_z_mm": 0.0,
        "target_w_deg": 0.0,
        "target_p_deg": 0.0,
        "target_r_deg": 0.0,
        "target_x_set": False,
        "target_y_set": False,
        "target_z_set": False,
        "target_w_set": False,
        "target_p_set": False,
        "target_r_set": False,
        "joint_1_deg": 0.0,
        "joint_2_deg": 0.0,
        "joint_3_deg": 0.0,
        "joint_4_deg": 0.0,
        "joint_5_deg": 0.0,
        "joint_6_deg": 0.0,
        "joint_velocity_percent": 0,
        "program_name": "",
    }
    data.update(overrides)
    return data


class FakeResponse:
    def __init__(self, body):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        return False

    def read(self, _size):
        return self.body


class FakeOpener:
    def __init__(self, decisions):
        self.decisions = list(decisions)
        self.requests = []

    def __call__(self, request, timeout):
        self.requests.append((request, timeout))
        decision = self.decisions.pop(0)
        envelope = {
            "message": {
                "role": "assistant",
                "content": json.dumps(decision),
            }
        }
        return FakeResponse(json.dumps(envelope).encode("utf-8"))


def test_local_ollama_request_uses_schema_and_zero_temperature():
    opener = FakeOpener([_decision()])
    planner = OllamaPlanner(PromptParser(), opener=opener)
    result = planner.interpret(
        "please move left side at 50 mm at 200 mm/s",
        "Driver state: CONNECTED",
    )

    assert result.plan is not None
    assert result.plan.delta_x_mm == -50.0
    assert result.plan.velocity_mm_s == 200
    request, timeout = opener.requests[0]
    payload = json.loads(request.data.decode("utf-8"))
    assert request.full_url == "http://127.0.0.1:11434/api/chat"
    assert timeout == 60.0
    assert payload["format"] == DECISION_SCHEMA
    assert payload["stream"] is False
    assert payload["think"] is False
    assert payload["options"]["temperature"] == 0
    system_prompt = payload["messages"][0]["content"]
    assert "DANGEROUS and NORMAL" in system_prompt
    assert "Do not omit normal objects" in system_prompt
    assert "never creates a motion action" in system_prompt


def test_robot_context_and_user_text_are_labelled_separately():
    opener = FakeOpener([_decision()])
    planner = OllamaPlanner(PromptParser(), opener=opener)
    planner.interpret("move left", "Current position: [1, 2, 3]")
    payload = json.loads(opener.requests[0][0].data.decode("utf-8"))
    final_message = payload["messages"][-1]["content"]
    assert "Trusted runtime robot context" in final_message
    assert "Untrusted user instruction" in final_message
    assert "Current position: [1, 2, 3]" in final_message


def test_conversation_history_is_bounded_and_sent_on_follow_up():
    opener = FakeOpener([_decision(), _decision(delta_x_mm=-10.0)])
    planner = OllamaPlanner(
        PromptParser(),
        history_turns=1,
        opener=opener,
    )
    planner.interpret("move left 50 mm", "state one")
    planner.interpret("do the same direction by 10 mm", "state two")
    second_payload = json.loads(opener.requests[1][0].data.decode("utf-8"))
    messages = second_payload["messages"]
    assert len(messages) == 4
    assert messages[1]["content"] == "move left 50 mm"
    assert messages[-1]["role"] == "user"


def test_non_motion_answer_has_no_plan():
    data = _decision(
        message="The current position is shown by the ROS client.",
        action="none",
        delta_x_mm=0.0,
        velocity_mm_s=0,
    )
    result = parse_model_decision(data, PromptParser())
    assert result.plan is None
    assert not result.needs_clarification


def test_absolute_cartesian_target_is_parsed():
    data = _decision(
        action="cartesian_target",
        delta_x_mm=0.0,
        velocity_mm_s=100,
        target_x_mm=-350.0,
        target_y_mm=800.0,
        target_z_mm=-190.0,
        target_w_deg=-179.0,
        target_p_deg=60.0,
        target_r_deg=-175.0,
        target_x_set=True,
        target_y_set=True,
        target_z_set=True,
        target_w_set=True,
        target_p_set=True,
        target_r_set=True,
    )
    result = parse_model_decision(data, PromptParser())
    assert isinstance(result.plan, CartesianTargetPlan)
    assert result.plan.target == (-350.0, 800.0, -190.0, -179.0, 60.0, -175.0)
    assert result.plan.velocity_mm_s == 100


def test_partial_absolute_cartesian_target_is_parsed():
    data = _decision(
        message="Interpreted request: set X to 300 mm.",
        action="cartesian_target",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        target_x_mm=300.0,
        target_x_set=True,
    )
    result = parse_model_decision(data, PromptParser())
    assert isinstance(result.plan, CartesianTargetPlan)
    assert result.plan.target == (300.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert result.plan.specified_mask == (
        True,
        False,
        False,
        False,
        False,
        False,
    )


def test_partial_target_rejects_value_for_omitted_axis():
    data = _decision(
        action="cartesian_target",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        target_x_mm=300.0,
        target_x_set=True,
        target_y_mm=400.0,
    )
    with pytest.raises(OllamaPlannerError, match="Omitted Cartesian Y"):
        parse_model_decision(data, PromptParser())


def test_cartesian_target_requires_at_least_one_selected_axis():
    data = _decision(
        action="cartesian_target",
        delta_x_mm=0.0,
        velocity_mm_s=0,
    )
    with pytest.raises(OllamaPlannerError, match="did not specify"):
        parse_model_decision(data, PromptParser())


def test_target_selector_must_be_boolean():
    data = _decision(target_x_set=1)
    with pytest.raises(OllamaPlannerError, match="not boolean"):
        parse_model_decision(data, PromptParser())


def test_absolute_cartesian_target_rejects_hidden_jog():
    data = _decision(
        action="cartesian_target",
        velocity_mm_s=100,
        target_x_mm=-350.0,
    )
    with pytest.raises(OllamaPlannerError, match="unused command"):
        parse_model_decision(data, PromptParser())


def test_absolute_joint_target_is_parsed_and_bounded():
    data = _decision(
        action="joint_target",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        joint_1_deg=10.0,
        joint_2_deg=20.0,
        joint_3_deg=-30.0,
        joint_4_deg=5.0,
        joint_5_deg=-5.0,
        joint_6_deg=15.0,
        joint_velocity_percent=5,
    )
    result = parse_model_decision(data, PromptParser())
    assert isinstance(result.plan, JointTargetPlan)
    assert result.plan.positions_deg == (10.0, 20.0, -30.0, 5.0, -5.0, 15.0)
    assert result.plan.velocity_percent == 5


def test_joint_target_above_velocity_cap_is_rejected():
    data = _decision(
        action="joint_target",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        joint_velocity_percent=11,
    )
    with pytest.raises(OllamaPlannerError, match="safety validation"):
        parse_model_decision(data, PromptParser())


def test_tp_program_is_normalized_and_other_fields_are_zero():
    data = _decision(
        action="tp_program",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        program_name="home_p",
    )
    result = parse_model_decision(data, PromptParser())
    assert isinstance(result.plan, TpProgramPlan)
    assert result.plan.program_name == "HOME_P"


def test_tp_program_rejects_motion_fields():
    data = _decision(
        action="tp_program",
        velocity_mm_s=0,
        program_name="HOME_P",
    )
    with pytest.raises(OllamaPlannerError, match="unused motion"):
        parse_model_decision(data, PromptParser())


def test_invalid_tp_program_name_is_rejected():
    data = _decision(
        action="tp_program",
        delta_x_mm=0.0,
        velocity_mm_s=0,
        program_name="BAD-NAME",
    )
    with pytest.raises(OllamaPlannerError, match="safety validation"):
        parse_model_decision(data, PromptParser())


def test_clarification_cannot_contain_motion():
    data = _decision(
        needs_clarification=True,
        clarification_question="Which rotation axis should I use?",
    )
    with pytest.raises(OllamaPlannerError, match="must not contain"):
        parse_model_decision(data, PromptParser())


def test_model_motion_above_local_limit_is_rejected():
    data = _decision(delta_x_mm=-51.0)
    with pytest.raises(OllamaPlannerError, match="safety validation"):
        parse_model_decision(data, PromptParser())


def test_non_motion_response_with_hidden_offset_is_rejected():
    data = _decision(action="none", velocity_mm_s=0)
    with pytest.raises(OllamaPlannerError, match="must be zero"):
        parse_model_decision(data, PromptParser())


def test_extra_model_fields_are_rejected():
    data = _decision(raw_command="unsafe")
    with pytest.raises(OllamaPlannerError, match="unsupported fields"):
        parse_model_decision(data, PromptParser())


def test_remote_endpoint_is_disabled_by_default():
    with pytest.raises(ValueError, match="Remote Ollama endpoints"):
        OllamaPlanner(
            PromptParser(),
            base_url="http://model.example.com:11434",
        )


def test_remote_endpoint_requires_explicit_opt_in():
    planner = OllamaPlanner(
        PromptParser(),
        base_url="https://model.example.com",
        allow_remote=True,
    )
    assert planner.url == "https://model.example.com/api/chat"


def test_connection_failure_has_actionable_error():
    def failing_opener(_request, timeout):
        raise URLError(f"offline after {timeout} seconds")

    planner = OllamaPlanner(PromptParser(), opener=failing_opener)
    with pytest.raises(OllamaPlannerError, match="Cannot reach local Ollama"):
        planner.interpret("move left", "state unavailable")


def test_malformed_model_json_is_rejected():
    class MalformedOpener:
        def __call__(self, _request, timeout):
            del timeout
            envelope = {"message": {"content": "not-json"}}
            return FakeResponse(json.dumps(envelope).encode("utf-8"))

    planner = OllamaPlanner(PromptParser(), opener=MalformedOpener())
    with pytest.raises(OllamaPlannerError, match="malformed"):
        planner.interpret("move left", "state unavailable")
