# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

import math
from types import SimpleNamespace

import pytest

from fanucpy_ros2_trajectory_controller.validation import (
    ValidatedTrajectoryPoint,
    select_execution_points,
    segment_velocity_percent,
    validate_goal_only_distance,
    validate_start_position,
    validate_trajectory,
)


JOINTS = tuple(f"joint_{index}" for index in range(1, 7))
LOWER = (-3.14, -1.57, -3.14, -3.31, -3.31, -6.28)
UPPER = (3.14, 2.79, 4.61, 3.31, 3.31, 6.28)
VELOCITY = (3.67, 3.32, 3.67, 6.98, 6.98, 10.47)


def duration(seconds, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def point(positions, seconds, velocities=(), accelerations=(), effort=()):
    return SimpleNamespace(
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
        effort=effort,
        time_from_start=duration(seconds),
    )


def trajectory(joint_names=JOINTS, points=None):
    return SimpleNamespace(
        joint_names=joint_names,
        points=(
            points
            if points is not None
            else [
                point((0.0,) * 6, 0),
                point((0.1,) * 6, 1, velocities=(0.1,) * 6),
            ]
        ),
    )


def validate(value):
    return validate_trajectory(
        value,
        JOINTS,
        LOWER,
        UPPER,
        VELOCITY,
        max_points=100,
        max_joint_step_rad=0.35,
    )


def test_valid_trajectory_is_reordered_to_driver_joint_order():
    reversed_names = tuple(reversed(JOINTS))
    result = validate(
        trajectory(
            joint_names=reversed_names,
            points=[point((6, 3, 2, 1, 0.5, 0.25), 0)],
        )
    )
    assert result.joint_names == JOINTS
    assert result.points[0].positions == (0.25, 0.5, 1.0, 2.0, 3.0, 6.0)


def test_goal_only_selects_exactly_the_final_validated_point():
    result = validate(
        trajectory(
            points=[
                point((0.0,) * 6, 0),
                point((0.1,) * 6, 1),
                point((0.2,) * 6, 2),
            ]
        )
    )
    selected = select_execution_points(result, "goal_only")
    assert len(selected) == 1
    assert selected[0][0] == 2
    assert selected[0][1] == result.points[-1]


def test_stop_at_waypoints_selects_every_validated_point():
    result = validate(trajectory())
    selected = select_execution_points(result, "stop_at_waypoints")
    assert [index for index, _point in selected] == [0, 1]

    with pytest.raises(ValueError, match="execution mode"):
        select_execution_points(result, "unsupported")


@pytest.mark.parametrize(
    "value, expected_message",
    [
        (
            trajectory(joint_names=("wrong",) + JOINTS[1:]),
            "do not match",
        ),
        (
            trajectory(points=[point((0.0,) * 6, 1), point((0.1,) * 6, 1)]),
            "strictly increase",
        ),
        (
            trajectory(points=[point((4.0, 0, 0, 0, 0, 0), 0)]),
            "position",
        ),
        (
            trajectory(
                points=[point((0.0,) * 6, 0), point((0.5,) * 6, 1)]
            ),
            "max_joint_step_rad",
        ),
        (
            trajectory(
                points=[point((0.0,) * 6, 0, velocities=(20.0,) * 6)]
            ),
            "velocity",
        ),
    ],
)
def test_invalid_trajectory_is_rejected(value, expected_message):
    with pytest.raises(ValueError, match=expected_message):
        validate(value)


def test_start_position_must_match_first_point():
    assert validate_start_position((0.0,) * 6, (0.01,) * 6, 0.05) == (0.01,) * 6
    with pytest.raises(ValueError, match="Trajectory start differs"):
        validate_start_position((0.0,) * 6, (0.1,) * 6, 0.05)


def test_goal_only_distance_is_bounded_per_joint():
    assert validate_goal_only_distance(
        (0.0,) * 6,
        (0.1, -0.2, 0.3, 0.0, 0.0, 0.0),
        0.35,
        JOINTS,
    ) == (0.1, -0.2, 0.3, 0.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="joint_3.*exceeding"):
        validate_goal_only_distance(
            (0.0,) * 6,
            (0.0, 0.0, 0.36, 0.0, 0.0, 0.0),
            0.35,
            JOINTS,
        )


@pytest.mark.parametrize("limit", [0.0, -0.1, math.nan])
def test_goal_only_distance_limit_must_be_positive_and_finite(limit):
    with pytest.raises(ValueError, match="maximum joint delta"):
        validate_goal_only_distance((0.0,) * 6, (0.1,) * 6, limit, JOINTS)


def test_segment_speed_uses_timing_and_respects_configured_cap():
    result = validate(trajectory())
    second = result.points[1]
    assert segment_velocity_percent(
        (0.0,) * 6,
        second,
        0.0,
        VELOCITY,
        default_percent=5,
        maximum_percent=25,
    ) == 4

    fast_point = ValidatedTrajectoryPoint(
        positions=(0.3,) * 6,
        velocities=(10.0,) * 6,
        accelerations=None,
        time_from_start_sec=0.1,
    )
    assert segment_velocity_percent(
        (0.0,) * 6,
        fast_point,
        0.0,
        VELOCITY,
        default_percent=5,
        maximum_percent=25,
    ) == 25
