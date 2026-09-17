# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import pytest

from fanucpy_ros2_assistant.vision_motion import (
    VisionMotionError,
    parse_vision_motion,
)
from fanucpy_ros2_assistant.workflows import (
    DROP_TARGET_INTENT,
    HOME_PROGRAM_INTENT,
    PICK_INTENT,
    build_named_cartesian_target,
    build_pick_motion_plan,
    is_workflow_question,
    pick_selection_prompt,
    special_intent,
    workflow_request_issue,
)
from fanucpy_ros2_task_planner.vision_context import VisionObservation


def _selection():
    observation = VisionObservation(
        track_id=8,
        label="pet-bottle-clear-food",
        is_danger=False,
        eye_to_hand_valid=True,
        eye_to_hand_x_mm=-820.130,
        eye_to_hand_y_mm=966.542,
    )
    selection = parse_vision_motion(
        "go to bottle one at 50 mm/s",
        (observation,),
        default_velocity_mm_s=25,
        max_velocity_mm_s=2000,
    )
    assert selection is not None
    return selection


@pytest.mark.parametrize(
    "prompt",
    [
        "pick battery one",
        "please pick up the second bottle",
        "could you grab track ID 8",
        "collect object one for me",
        "retrieve the cardboard",
    ],
)
def test_rough_pick_intents_are_recognized(prompt):
    assert special_intent(prompt) == PICK_INTENT


@pytest.mark.parametrize(
    "prompt",
    [
        "go to the drop",
        "take it to the drop position",
        "please move the robot to our drop-off zone",
        "bring this object to the drop location",
        "drop it at 200 mm/s",
        "could you drop it at a speed of 200 millimeters per second?",
        "please drop off the object at max speed",
        "head to the saved drop position at full speed",
        "drop now",
        "drop",
        "go to drop position as fast as you can",
    ],
)
def test_rough_drop_target_intents_are_recognized(prompt):
    assert special_intent(prompt) == DROP_TARGET_INTENT
    assert workflow_request_issue(prompt, DROP_TARGET_INTENT) is None


@pytest.mark.parametrize(
    "prompt",
    [
        "go home",
        "return the robot to home position",
        "please take the robot back home",
        "run HOME_P",
        "home",
        "back home please",
        "could you head home now?",
    ],
)
def test_rough_home_intents_are_recognized(prompt):
    assert special_intent(prompt) == HOME_PROGRAM_INTENT
    assert workflow_request_issue(prompt, HOME_PROGRAM_INTENT) is None


def test_bare_drop_now_means_saved_pose_not_gripper_release():
    assert special_intent("drop it") == DROP_TARGET_INTENT
    assert workflow_request_issue("drop it", DROP_TARGET_INTENT) is None


@pytest.mark.parametrize(
    "prompt",
    [
        "don't drop it",
        "please don’t go home",
        "do not pick battery one",
        "stop and go home",
        "drop it then go home",
        "pick battery one and drop it",
        "drop it if the bottle is gone",
        "drop it after I move the bottle",
        "drop it here",
        "drop it at the battery",
        "drop it at X 500 Y 700",
        "go home and run OTHER_P",
        "drop it and open the gripper",
        "go home at 200 mm/s",
        "go home at max speed",
        "go home quickly",
        "drop it at 20 percent",
        "drop it slowly",
    ],
)
def test_named_workflows_reject_unhandled_instructions(prompt):
    intent = special_intent(prompt)
    assert intent is not None
    assert workflow_request_issue(prompt, intent) is not None


@pytest.mark.parametrize(
    "prompt",
    [
        "where is the drop position?",
        "what happens if I go home?",
        "tell me about the pick motion",
    ],
)
def test_informational_wording_is_not_execution_permission(prompt):
    assert is_workflow_question(prompt)


def test_polite_command_is_not_a_workflow_question():
    assert not is_workflow_question("could you drop it at 200 mm/s?")


def test_pick_wording_is_rewritten_only_for_object_selection():
    assert pick_selection_prompt("please pick up battery two at 25 mm/s") == (
        "please go to battery two at 25 mm/s"
    )
    with pytest.raises(VisionMotionError, match="does not contain"):
        pick_selection_prompt("go to battery two")


def test_pick_plan_uses_exact_configured_z_and_preserves_orientation():
    plan = build_pick_motion_plan(
        selection=_selection(),
        current_values=(-458.0, 1201.0, -190.0, -179.0, 60.0, -175.0),
        xy_offset_mm=(0.0, 0.0),
        approach_z_mm=-190.0,
        descend_z_mm=-238.0,
        retract_z_mm=-190.0,
        max_xy_travel_mm=500.0,
        max_approach_z_change_mm=100.0,
        max_vertical_stage_mm=50.0,
        max_velocity_mm_s=2000,
    )
    assert plan.approach.target == (
        -820.130,
        966.542,
        -190.0,
        -179.0,
        60.0,
        -175.0,
    )
    assert plan.descend.target == (
        -820.130,
        966.542,
        -238.0,
        -179.0,
        60.0,
        -175.0,
    )
    assert plan.retract.target == plan.approach.target
    assert plan.approach.velocity_mm_s == 50


def test_pick_plan_rejects_wrong_direction_and_excessive_vertical_stage():
    common = {
        "selection": _selection(),
        "current_values": (-820.0, 966.0, -190.0, -179.0, 60.0, -175.0),
        "xy_offset_mm": (0.0, 0.0),
        "approach_z_mm": -190.0,
        "retract_z_mm": -190.0,
        "max_xy_travel_mm": 500.0,
        "max_approach_z_change_mm": 100.0,
        "max_vertical_stage_mm": 50.0,
        "max_velocity_mm_s": 2000,
    }
    with pytest.raises(VisionMotionError, match="below"):
        build_pick_motion_plan(descend_z_mm=-180.0, **common)
    with pytest.raises(VisionMotionError, match="vertical stage"):
        build_pick_motion_plan(descend_z_mm=-241.0, **common)


def test_named_drop_target_is_exact_and_velocity_bounded():
    target = (237.0, 720.0, -167.0, -179.0, 60.0, -175.0)
    plan = build_named_cartesian_target(target, 25, 2000)
    assert plan.target == target
    with pytest.raises(VisionMotionError, match="range"):
        build_named_cartesian_target(target, 2001, 2000)
