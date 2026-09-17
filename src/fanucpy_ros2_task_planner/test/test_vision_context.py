# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import math

from fanucpy_ros2_task_planner.vision_context import (
    VisionObservation,
    coordinate_question_answer,
    is_vision_observation_request,
    model_vision_context,
)


def test_rough_vision_questions_are_identified():
    assert is_vision_observation_request("what are you see?")
    assert is_vision_observation_request("what is visible on the conveyor?")
    assert is_vision_observation_request("what did the camera detect?")
    assert is_vision_observation_request("coordinates of battery one")
    assert is_vision_observation_request("projected pixel for bottle 2")
    assert is_vision_observation_request("eye-to-hand location of track 12")
    assert is_vision_observation_request("where is battery four?")
    assert not is_vision_observation_request("move left by 10 mm")


def test_fresh_vision_counts_exact_labels():
    context = model_vision_context(
        observations=(
            VisionObservation(7, "battery", True),
            VisionObservation(9, "battery", True),
            VisionObservation(11, "pet-bottle-clear-food", False),
        ),
        age_sec=0.2,
        maximum_age_sec=1.0,
        source="ultralytics_model_tracker",
        frame_sequence=42,
    )
    assert "Vision state: FRESH" in context
    assert "Visible DANGEROUS exact-label counts" in context
    assert "- battery: 2" in context
    assert "Visible NORMAL exact-label counts" in context
    assert "- pet-bottle-clear-food: 1" in context
    assert "Vision frame sequence: 42" in context


def test_fresh_empty_vision_is_an_observed_empty_scene():
    context = model_vision_context(
        observations=(),
        age_sec=0.1,
        maximum_age_sec=1.0,
    )
    assert "Vision state: FRESH" in context
    assert "Visible DANGEROUS exact-label counts: none" in context
    assert "Visible NORMAL exact-label counts: none" in context
    assert "Visible object records: none" in context


def test_stale_vision_must_not_be_reported_as_current():
    context = model_vision_context(
        observations=(VisionObservation(7, "battery", True),),
        age_sec=1.1,
        maximum_age_sec=1.0,
    )
    assert "Vision state: STALE" in context
    assert "Do not report these objects as currently visible" in context
    assert "battery" not in context


def test_unavailable_vision_is_not_an_empty_scene():
    context = model_vision_context(
        observations=None,
        age_sec=math.inf,
        maximum_age_sec=1.0,
    )
    assert "Vision state: UNAVAILABLE" in context
    assert "Do not claim that the scene is empty" in context


def test_object_aliases_and_both_coordinate_systems_are_formatted():
    context = model_vision_context(
        observations=(
            VisionObservation(
                track_id=20,
                label="battery",
                is_danger=True,
                projected_center_valid=True,
                projected_u_px=600.25,
                projected_v_px=300.5,
                eye_to_hand_valid=True,
                eye_to_hand_x_mm=-350.125,
                eye_to_hand_y_mm=800.75,
            ),
            VisionObservation(
                track_id=10,
                label="battery",
                is_danger=True,
                projected_center_valid=True,
                projected_u_px=200.0,
                projected_v_px=100.0,
                eye_to_hand_valid=True,
                eye_to_hand_x_mm=-250.0,
                eye_to_hand_y_mm=700.0,
            ),
            VisionObservation(
                track_id=30,
                label="pet-bottle-clear-food",
                is_danger=False,
            ),
        ),
        age_sec=0.1,
        maximum_age_sec=1.0,
        eye_to_hand_frame_id="fanuc_world",
        eye_to_hand_calibration="phase_two_active_affine",
    )
    first = context.index('aliases="object 1", "battery 1"')
    second = context.index('aliases="object 2", "battery 2"')
    assert first < second
    assert "track_id=10" in context[first:second]
    assert "projected_px=(u=200.00, v=100.00) px" in context
    assert "eye_to_hand[fanuc_world]=(X=-250.000, Y=700.000) mm" in context
    assert "calibration=phase_two_active_affine" in context
    assert '"pet-bottle-clear-food 1", "bottle 1"' in context
    assert "projected_px=UNAVAILABLE" in context
    assert "eye_to_hand=UNAVAILABLE; Z=NOT_AVAILABLE" in context


def test_non_finite_coordinate_is_not_exposed_as_a_valid_measurement():
    context = model_vision_context(
        observations=(
            VisionObservation(
                track_id=1,
                label="battery",
                is_danger=True,
                projected_center_valid=True,
                projected_u_px=math.nan,
                projected_v_px=100.0,
                eye_to_hand_valid=True,
                eye_to_hand_x_mm=math.inf,
                eye_to_hand_y_mm=700.0,
            ),
        ),
        age_sec=0.1,
        maximum_age_sec=1.0,
    )
    assert "projected_px=UNAVAILABLE" in context
    assert "eye_to_hand=UNAVAILABLE" in context
    assert "nan" not in context.lower()
    assert "inf" not in context.lower()


def _coordinate_observations():
    return (
        VisionObservation(
            track_id=20,
            label="battery",
            is_danger=True,
            projected_center_valid=True,
            projected_u_px=600.25,
            projected_v_px=300.5,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-780.23996035,
            eye_to_hand_y_mm=904.84307554,
        ),
        VisionObservation(
            track_id=10,
            label="battery",
            is_danger=True,
            projected_center_valid=True,
            projected_u_px=200.0,
            projected_v_px=100.0,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-475.33941635,
            eye_to_hand_y_mm=1107.01406254,
        ),
        VisionObservation(
            track_id=30,
            label="pet-bottle-clear-food",
            is_danger=False,
            projected_center_valid=True,
            projected_u_px=800.0,
            projected_v_px=500.0,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-1287.85910735,
            eye_to_hand_y_mm=601.35812354,
        ),
        VisionObservation(
            track_id=40,
            label="cardboard",
            is_danger=False,
            projected_center_valid=False,
            eye_to_hand_valid=False,
        ),
    )


def _answer(prompt):
    return coordinate_question_answer(
        prompt=prompt,
        observations=_coordinate_observations(),
        age_sec=0.1,
        maximum_age_sec=1.0,
        eye_to_hand_frame_id="fanuc_world",
        eye_to_hand_calibration="phase_two_active_affine",
    )


def test_coordinate_question_selects_battery_one_by_track_order():
    answer = _answer("please tell me the coordinates of battery one")
    assert answer is not None
    assert "Battery 1 (battery; dangerous; track ID 10)" in answer
    assert "(u=200.00, v=100.00) px" in answer
    assert "(X=-475.339, Y=1107.014) mm in fanuc_world" in answer
    assert "using phase_two_active_affine" in answer
    assert "does not provide Z" in answer


def test_coordinate_question_can_request_one_coordinate_system():
    projected = _answer("projected pixel coordinates of battery two")
    assert projected is not None
    assert "track ID 20" in projected
    assert "(u=600.25, v=300.50) px" in projected
    assert "eye-to-hand" not in projected

    eye = _answer("where are the eye-to-hand coordinates of bottle 1?")
    assert eye is not None
    assert "Bottle 1 (pet-bottle-clear-food; normal; track ID 30)" in eye
    assert "(X=-1287.859, Y=601.358) mm" in eye
    assert "projected center" not in eye


def test_plural_and_arbitrary_class_coordinate_questions():
    batteries = _answer("tell me the coordinates of all batteries")
    assert batteries is not None
    assert batteries.count("Battery ") == 2
    assert batteries.index("track ID 10") < batteries.index("track ID 20")

    cardboard = _answer("coordinates of cardboard one")
    assert cardboard is not None
    assert "Cardboard 1" in cardboard
    assert "projected center is unavailable" in cardboard
    assert "eye-to-hand position is unavailable" in cardboard


def test_missing_ordinal_and_non_object_where_question_are_handled():
    missing = _answer("where is battery four?")
    assert missing is not None
    assert "only 2 battery object(s)" in missing
    assert "battery 4 is unavailable" in missing
    assert _answer("where is the current robot?") is None


def test_coordinate_question_rejects_stale_or_unavailable_vision():
    stale = coordinate_question_answer(
        prompt="coordinates of battery one",
        observations=_coordinate_observations(),
        age_sec=2.0,
        maximum_age_sec=1.0,
    )
    assert stale is not None
    assert "vision frame is stale" in stale
    unavailable = coordinate_question_answer(
        prompt="coordinates of battery one",
        observations=None,
        age_sec=math.inf,
        maximum_age_sec=1.0,
    )
    assert unavailable is not None
    assert "coordinates are unavailable" in unavailable
