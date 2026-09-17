# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import math

import pytest

from fanucpy_ros2_task_planner.prompt_parser import PromptParseError
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    JointTargetPlan,
    TpProgramPlan,
    joint_motion_duration_sec,
    normalize_tp_program_name,
    resolve_cartesian_target,
    validate_cartesian_target_bounds,
    validate_cartesian_target_plan,
    validate_joint_target_plan,
    validate_tp_program_plan,
)


LOWER = (-3.14, -1.57, -3.14, -3.31, -3.31, -6.28)
UPPER = (3.14, 2.79, 4.61, 3.31, 3.31, 6.28)


def test_absolute_cartesian_target_is_not_limited_by_jog_step():
    plan = CartesianTargetPlan(-350, 800, -190, -179, 60, -175, 200)
    assert validate_cartesian_target_plan(plan, 2000) == plan


def test_optional_absolute_cartesian_bounds_are_enforced():
    plan = CartesianTargetPlan(-350, 800, -190, -179, 60, -175, 200)
    assert validate_cartesian_target_bounds(
        plan,
        (-1000, -1000, -1000, -360, -360, -360),
        (1000, 1000, 1000, 360, 360, 360),
    ) == plan
    with pytest.raises(PromptParseError, match="Y target"):
        validate_cartesian_target_bounds(
            plan,
            (-500, -500, -500, -360, -360, -360),
            (500, 500, 500, 360, 360, 360),
        )


def test_partial_absolute_target_preserves_unspecified_live_axes():
    plan = CartesianTargetPlan(
        300,
        0,
        -190,
        0,
        0,
        0,
        velocity_mm_s=200,
        x_set=True,
        y_set=False,
        z_set=True,
        w_set=False,
        p_set=False,
        r_set=False,
    )
    resolved = resolve_cartesian_target(
        plan,
        (-350, 990.080, -6.832, -179, 60, -175),
    )
    assert resolved.target == (300, 990.080, -190, -179, 60, -175)
    assert resolved.is_complete
    assert resolved.velocity_mm_s == 200


def test_partial_absolute_target_needs_one_explicit_axis():
    plan = CartesianTargetPlan(
        0,
        0,
        0,
        0,
        0,
        0,
        x_set=False,
        y_set=False,
        z_set=False,
        w_set=False,
        p_set=False,
        r_set=False,
    )
    with pytest.raises(PromptParseError, match="at least one axis"):
        validate_cartesian_target_plan(plan, 2000)


def test_bounds_reject_unresolved_partial_target():
    plan = CartesianTargetPlan(
        300,
        0,
        0,
        0,
        0,
        0,
        x_set=True,
        y_set=False,
        z_set=False,
        w_set=False,
        p_set=False,
        r_set=False,
    )
    with pytest.raises(PromptParseError, match="must be resolved"):
        validate_cartesian_target_bounds(
            plan,
            (-1000, -1000, -1000, -360, -360, -360),
            (1000, 1000, 1000, 360, 360, 360),
        )


def test_joint_target_validates_position_velocity_and_delta():
    plan = JointTargetPlan(5, 5, -5, 5, -5, 5, velocity_percent=5)
    target = validate_joint_target_plan(
        plan,
        LOWER,
        UPPER,
        max_velocity_percent=10,
        current_positions_rad=(0, 0, 0, 0, 0, 0),
        max_delta_rad=0.35,
    )
    assert target[0] == pytest.approx(math.radians(5))


def test_joint_target_over_delta_limit_is_rejected():
    plan = JointTargetPlan(21, 0, 0, 0, 0, 0, velocity_percent=5)
    with pytest.raises(PromptParseError, match="J1 command delta"):
        validate_joint_target_plan(
            plan,
            LOWER,
            UPPER,
            max_velocity_percent=10,
            current_positions_rad=(0, 0, 0, 0, 0, 0),
            max_delta_rad=0.35,
        )


def test_joint_motion_duration_requests_selected_percentage():
    duration = joint_motion_duration_sec(
        (0, 0, 0, 0, 0, 0),
        (0.1, 0, 0, 0, 0, 0),
        (1, 1, 1, 1, 1, 1),
        velocity_percent=10,
    )
    assert duration == pytest.approx(1.0)


def test_tp_program_name_matches_driver_contract():
    assert normalize_tp_program_name(" home_p ") == "HOME_P"
    assert validate_tp_program_plan(TpProgramPlan("test_1")).program_name == "TEST_1"
    with pytest.raises(PromptParseError, match="1..32"):
        normalize_tp_program_name("HOME-P")
