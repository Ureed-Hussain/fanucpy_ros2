# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import pytest

from fanucpy_ros2_task_planner.prompt_parser import (
    CartesianJogPlan,
    PromptParseError,
    PromptParser,
    format_plan,
    validate_jog_plan,
)


@pytest.fixture
def parser():
    return PromptParser()


def test_direction_defaults_use_documented_user_frame_mapping(parser):
    assert parser.parse("move left").offset == (
        -10.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert parser.parse("go to the right 5 mm").delta_x_mm == 5.0
    assert parser.parse("up").delta_z_mm == 10.0
    assert parser.parse("backwards 2 mm").delta_y_mm == -2.0


def test_centimetres_are_converted_to_millimetres(parser):
    plan = parser.parse("move forward 1.5 cm")
    assert plan.delta_y_mm == 15.0


def test_compound_translation_and_rotation(parser):
    plan = parser.parse(
        "move down 5 mm and rotate pitch negative 1 degree at 20 mm/s"
    )
    assert plan.offset == (0.0, 0.0, -5.0, 0.0, -1.0, 0.0)
    assert plan.velocity_mm_s == 20


def test_roll_pitch_and_yaw_map_to_w_p_and_r(parser):
    assert parser.parse("rotate roll positive 1 degree").delta_w_deg == 1.0
    assert parser.parse("rotate pitch negative 1 degree").delta_p_deg == -1.0
    assert parser.parse("rotate yaw 0.5 degrees").delta_r_deg == 0.5


def test_clockwise_defaults_to_negative_yaw(parser):
    plan = parser.parse("rotate clockwise 0.5 degrees")
    assert plan.delta_r_deg == -0.5


def test_clockwise_can_name_an_axis(parser):
    plan = parser.parse("rotate clockwise 0.5 degrees around x")
    assert plan.delta_w_deg == -0.5


def test_counterclockwise_hyphen_is_normalized(parser):
    plan = parser.parse("rotate counter-clockwise 0.5 degrees around z")
    assert plan.delta_r_deg == 0.5


def test_velocity_is_optional_and_zero_selects_driver_default(parser):
    assert parser.parse("move up 5 mm").velocity_mm_s == 0
    plan = parser.parse("move up 5 mm with a speed of 25 mm/s")
    assert plan.velocity_mm_s == 25


@pytest.mark.parametrize(
    "prompt",
    [
        "",
        "pick up the battery",
        "move somewhere left",
        "move left -5 mm",
        "rotate a little",
        "move left 1 inch",
        "move left 5 mm and right 2 mm",
        "move left 5 mm at 20 mm/s at 30 mm/s",
    ],
)
def test_unsupported_or_ambiguous_prompts_are_rejected(parser, prompt):
    with pytest.raises(PromptParseError):
        parser.parse(prompt)


def test_translation_above_limit_is_rejected(parser):
    with pytest.raises(PromptParseError, match="per-axis limit"):
        parser.parse("move left 51 mm")


def test_rotation_above_limit_is_rejected(parser):
    with pytest.raises(PromptParseError, match="per-axis limit"):
        parser.parse("rotate yaw 3 degrees")


def test_velocity_above_limit_is_rejected(parser):
    with pytest.raises(PromptParseError, match="within"):
        parser.parse("move up 5 mm at 2001 mm/s")


def test_live_driver_limits_can_be_stricter_than_dry_run_limits(parser):
    plan = parser.parse("move right 40 mm")
    with pytest.raises(PromptParseError, match="20 mm"):
        validate_jog_plan(plan, 20.0, 2.0, 2000)


def test_plan_preview_has_units_frame_and_driver_default(parser):
    preview = format_plan(parser.parse("move up 5 mm"), "fanuc_world")
    assert "Frame: fanuc_world" in preview
    assert "[mm, deg]" in preview
    assert "Velocity: driver default" in preview


def test_zero_plan_is_rejected():
    with pytest.raises(PromptParseError, match="non-zero"):
        validate_jog_plan(
            CartesianJogPlan(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            50.0,
            2.0,
            2000,
        )
