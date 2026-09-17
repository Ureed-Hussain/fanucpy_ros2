# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import math

import pytest

from fanucpy_ros2_assistant.vision_motion import (
    VisionMotionError,
    build_vision_cartesian_target,
    looks_like_vision_motion,
    parse_vision_motion,
    select_visible_track,
    target_xy_shift_mm,
)
from fanucpy_ros2_task_planner.vision_context import VisionObservation


def _observations():
    return (
        VisionObservation(
            track_id=20,
            label="battery",
            is_danger=True,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-780.0,
            eye_to_hand_y_mm=905.0,
        ),
        VisionObservation(
            track_id=10,
            label="battery",
            is_danger=True,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-475.0,
            eye_to_hand_y_mm=1107.0,
        ),
        VisionObservation(
            track_id=30,
            label="pet-bottle-clear-food",
            is_danger=False,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-1288.0,
            eye_to_hand_y_mm=601.0,
        ),
        VisionObservation(
            track_id=40,
            label="cardboard",
            is_danger=False,
            eye_to_hand_valid=True,
            eye_to_hand_x_mm=-300.0,
            eye_to_hand_y_mm=750.0,
        ),
    )


def _parse(prompt):
    return parse_vision_motion(
        prompt=prompt,
        observations=_observations(),
        default_velocity_mm_s=25,
        max_velocity_mm_s=2000,
    )


def test_recognizes_object_motion_but_not_existing_absolute_command():
    assert looks_like_vision_motion("go to battery one", _observations())
    assert looks_like_vision_motion("pick bottle one", _observations())
    assert not looks_like_vision_motion("move X 300", _observations())
    assert not looks_like_vision_motion("move left by 10 mm", _observations())


def test_selects_battery_ordinals_in_track_id_order():
    first = _parse("please go to battery one")
    second = _parse("align with the second battery at 200 mm/s")
    assert first is not None
    assert second is not None
    assert first.alias.observation.track_id == 10
    assert first.velocity_mm_s == 25
    assert second.alias.observation.track_id == 20
    assert second.velocity_mm_s == 200


def test_selects_bottle_arbitrary_label_object_and_track():
    bottle = _parse("move above bottle 1")
    cardboard = _parse("go to cardboard")
    track = _parse("position over track ID 30")
    assert bottle is not None
    assert cardboard is not None
    assert track is not None
    assert bottle.alias.observation.label == "pet-bottle-clear-food"
    assert cardboard.alias.observation.track_id == 40
    assert track.alias.observation.track_id == 30


def test_ambiguous_missing_and_unsupported_requests_are_rejected():
    with pytest.raises(VisionMotionError, match="Which battery"):
        _parse("go to the battery")
    with pytest.raises(VisionMotionError, match="Only 2 battery"):
        _parse("go to battery four")
    with pytest.raises(VisionMotionError, match="not enabled"):
        _parse("pick battery one")
    with pytest.raises(VisionMotionError, match="left/right"):
        _parse("move left of battery one")
    with pytest.raises(VisionMotionError, match="distance offsets"):
        _parse("go to battery one with 20 mm offset")


def test_unrelated_existing_task_is_delegated():
    assert _parse("go to X -350 at 100 mm/s") is None
    assert _parse("run TP program HOME_P") is None


def test_generic_object_requires_unique_selection_or_an_ordinal():
    one = parse_vision_motion(
        "go to the object",
        (_observations()[0],),
        default_velocity_mm_s=25,
        max_velocity_mm_s=2000,
    )
    assert one is not None
    assert one.alias.observation.track_id == 20
    with pytest.raises(VisionMotionError, match="Which object"):
        _parse("go to the object")
    third = _parse("go to object three")
    assert third is not None
    assert third.alias.observation.track_id == 30


def test_target_preserves_current_z_and_orientation_and_applies_xy_offset():
    selection = _parse("go to battery one at 100 mm/s")
    assert selection is not None
    target = build_vision_cartesian_target(
        selection=selection,
        current_values=(-450.0, 1080.0, -190.0, -179.0, 60.0, -175.0),
        xy_offset_mm=(5.0, -2.0),
        max_xy_travel_mm=100.0,
        max_velocity_mm_s=2000,
    )
    assert target.partial_plan.specified_mask == (
        True,
        True,
        False,
        False,
        False,
        False,
    )
    assert target.resolved_plan.target == (
        -470.0,
        1105.0,
        -190.0,
        -179.0,
        60.0,
        -175.0,
    )
    assert math.isclose(target.xy_travel_mm, math.hypot(20.0, 25.0))


def test_invalid_geometry_and_excessive_travel_are_rejected():
    invalid = VisionObservation(
        track_id=1,
        label="battery",
        is_danger=True,
        eye_to_hand_valid=False,
    )
    selection = select_visible_track((invalid,), 1, "battery", 25)
    with pytest.raises(VisionMotionError, match="no valid eye-to-hand"):
        build_vision_cartesian_target(
            selection,
            current_values=(0.0, 0.0, 1.0, 2.0, 3.0, 4.0),
            xy_offset_mm=(0.0, 0.0),
            max_xy_travel_mm=100.0,
            max_velocity_mm_s=2000,
        )

    valid = _parse("go to battery one")
    assert valid is not None
    with pytest.raises(VisionMotionError, match="vision-motion limit"):
        build_vision_cartesian_target(
            valid,
            current_values=(0.0, 0.0, 1.0, 2.0, 3.0, 4.0),
            xy_offset_mm=(0.0, 0.0),
            max_xy_travel_mm=10.0,
            max_velocity_mm_s=2000,
        )


def test_refresh_shift_is_measured_in_xy():
    selection = _parse("go to battery one")
    assert selection is not None
    first = build_vision_cartesian_target(
        selection,
        current_values=(-475.0, 1107.0, 10.0, 20.0, 30.0, 40.0),
        xy_offset_mm=(0.0, 0.0),
        max_xy_travel_mm=100.0,
        max_velocity_mm_s=2000,
    )
    moved = VisionObservation(
        track_id=10,
        label="battery",
        is_danger=True,
        eye_to_hand_valid=True,
        eye_to_hand_x_mm=-472.0,
        eye_to_hand_y_mm=1111.0,
    )
    refreshed_selection = select_visible_track((moved,), 10, "battery", 25)
    second = build_vision_cartesian_target(
        refreshed_selection,
        current_values=(-475.0, 1107.0, 10.0, 20.0, 30.0, 40.0),
        xy_offset_mm=(0.0, 0.0),
        max_xy_travel_mm=100.0,
        max_velocity_mm_s=2000,
    )
    assert math.isclose(target_xy_shift_mm(first, second), 5.0)


def test_refresh_rejects_changed_identity_source():
    observation = VisionObservation(
        track_id=10,
        label="battery",
        is_danger=True,
        eye_to_hand_valid=True,
        eye_to_hand_x_mm=-475.0,
        eye_to_hand_y_mm=1107.0,
        identity_source="fallback",
    )
    with pytest.raises(VisionMotionError, match="identity source"):
        select_visible_track(
            (observation,),
            track_id=10,
            expected_label="battery",
            velocity_mm_s=25,
            expected_identity_source="bytetrack",
        )
