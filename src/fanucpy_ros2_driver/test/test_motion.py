# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

import math

import pytest

from fanucpy_ros2_driver.motion import (
    validate_cartesian_offset,
    validate_cartesian_target,
    validate_cartesian_velocity,
)


def test_valid_cartesian_offset_is_returned_as_floats():
    assert validate_cartesian_offset((1, 0, -1, 0, 0.5, 0), 5.0, 2.0) == (
        1.0,
        0.0,
        -1.0,
        0.0,
        0.5,
        0.0,
    )


@pytest.mark.parametrize(
    "offset",
    [
        (0, 0, 0, 0, 0, 0),
        (5.1, 0, 0, 0, 0, 0),
        (0, 0, 0, 0, 2.1, 0),
        (math.nan, 0, 0, 0, 0, 0),
    ],
)
def test_invalid_cartesian_offset_is_rejected(offset):
    with pytest.raises(ValueError):
        validate_cartesian_offset(offset, 5.0, 2.0)


def test_zero_velocity_selects_driver_default():
    assert validate_cartesian_velocity(0, 25, 250) == 25


def test_requested_velocity_is_limited_by_driver():
    assert validate_cartesian_velocity(100, 25, 250) == 100
    with pytest.raises(ValueError):
        validate_cartesian_velocity(251, 25, 250)


def test_absolute_target_has_no_relative_step_limit():
    target = (-350, 800, -190, -179, 60, -175)
    assert validate_cartesian_target(target) == tuple(float(x) for x in target)


def test_optional_absolute_target_bounds_are_enforced():
    target = (-350, 800, -190, -179, 60, -175)
    assert validate_cartesian_target(
        target,
        enforce_bounds=True,
        lower_bounds=(-1000, -1000, -1000, -360, -360, -360),
        upper_bounds=(1000, 1000, 1000, 360, 360, 360),
    ) == tuple(float(x) for x in target)
    with pytest.raises(ValueError, match="Y target"):
        validate_cartesian_target(
            target,
            enforce_bounds=True,
            lower_bounds=(-500, -500, -500, -360, -360, -360),
            upper_bounds=(500, 500, 500, 360, 360, 360),
        )
