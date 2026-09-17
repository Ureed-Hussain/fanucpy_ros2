# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

from fanucpy_ros2_task_planner.conversation import (
    CartesianSnapshot,
    JointSnapshot,
    format_absolute_cartesian_target,
    format_partial_cartesian_target,
    format_joint_snapshot,
    format_joint_target,
    format_snapshot,
    format_target,
    model_robot_context,
)
from fanucpy_ros2_task_planner.prompt_parser import CartesianJogPlan
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    JointTargetPlan,
)


def test_target_preview_adds_relative_offsets():
    snapshot = CartesianSnapshot(
        frame_id="fanuc_world",
        x_mm=100.0,
        y_mm=200.0,
        z_mm=300.0,
        w_deg=10.0,
        p_deg=20.0,
        r_deg=30.0,
    )
    plan = CartesianJogPlan(
        -50.0,
        5.0,
        0.0,
        0.0,
        -1.0,
        0.5,
        velocity_mm_s=200,
    )
    assert snapshot.target_for(plan) == (
        50.0,
        205.0,
        300.0,
        10.0,
        19.0,
        30.5,
    )


def test_operator_output_separates_current_and_target():
    snapshot = CartesianSnapshot("fanuc_world", 1, 2, 3, 4, 5, 6)
    plan = CartesianJogPlan(5, 0, 0, 0, 0, 0)
    assert "Current [fanuc_world]" in format_snapshot(snapshot)
    target = format_target(snapshot, plan)
    assert "Calculated target [fanuc_world]" in target
    assert "[6.000, 2.000, 3.000" in target


def test_missing_state_never_invents_target():
    plan = CartesianJogPlan(5, 0, 0, 0, 0, 0)
    assert "unavailable" in format_target(None, plan)


def test_model_context_contains_only_required_robot_facts():
    snapshot = CartesianSnapshot("fanuc_world", 1, 2, 3, 4, 5, 6)
    joints = JointSnapshot(
        ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
        (0.0, 0.1, 0.2, 0.3, 0.4, 0.5),
    )
    context = model_robot_context(
        snapshot=snapshot,
        driver_state="CONNECTED",
        motion_enabled=True,
        frame_id="fanuc_world",
        max_translation_step_mm=50.0,
        max_rotation_step_deg=2.0,
        max_velocity_mm_s=2000,
        joint_snapshot=joints,
        program_execution_enabled=True,
        allowed_tp_programs=("HOME_P",),
        absolute_cartesian_enabled=True,
        absolute_cartesian_bounds_enabled=False,
    )
    assert "Driver state: CONNECTED" in context
    assert "Motion gate enabled: True" in context
    assert "Current [fanuc_world]" in context
    assert "50 mm" in context
    assert "2000 mm/s" in context
    assert "Current joints" in context
    assert "TP program gate enabled: True" in context
    assert "Allowed TP programs: HOME_P" in context
    assert "Direct absolute Cartesian gate enabled: True" in context


def test_absolute_and_joint_targets_have_independent_formatters():
    cartesian = CartesianTargetPlan(1, 2, 3, 4, 5, 6, 20)
    assert "Absolute target [fanuc_world]" in format_absolute_cartesian_target(
        "fanuc_world",
        cartesian,
    )
    joints = JointSnapshot(
        ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
        (0, 0, 0, 0, 0, 0),
    )
    assert "Current joints" in format_joint_snapshot(joints)
    target = JointTargetPlan(1, 2, 3, 4, 5, 6, 5)
    assert "Absolute joint target" in format_joint_target(joints.names, target)


def test_partial_cartesian_formatter_identifies_preserved_axes():
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
    text = format_partial_cartesian_target("fanuc_world", plan)
    assert "X=300.000 mm" in text
    assert "preserve current Y/Z/W/P/R" in text
