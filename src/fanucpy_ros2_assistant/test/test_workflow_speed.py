# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import pytest

from fanucpy_ros2_assistant.vision_motion import (
    VisionMotionError,
    requested_cartesian_velocity,
)


@pytest.mark.parametrize(
    "prompt, expected",
    [
        ("drop it", 25),
        ("drop it at 200 mm/s", 200),
        ("please drop it at 200mm/s", 200),
        ("drop it at a speed of 200 millimeters per second", 200),
        ("drop it with speed=200 mm/sec", 200),
        ("pick battery one at 50 millimetres/second", 50),
        ("go to bottle one at 200.0 mm/s", 200),
        ("drop it at max speed", 325),
        ("go to drop position at maximum speed", 325),
        ("please drop it at full speed", 325),
        ("drop it at the highest allowed speed", 325),
        ("drop it as fast as possible", 325),
        ("drop it as fast as you can", 325),
    ],
)
def test_explicit_or_default_speed_is_resolved(prompt, expected):
    assert requested_cartesian_velocity(prompt, 25, 325) == expected


@pytest.mark.parametrize(
    "prompt",
    [
        "drop it at 0 mm/s",
        "drop it at -200 mm/s",
        "drop it at 200.5 mm/s",
        "drop it at 326 mm/s",
        "drop it at 200 mm/s at max speed",
        "drop it at 200 mm/s or 100 mm/s",
        "drop it at max speed or full speed",
        "drop it at 200 mm",
        "drop it at 200",
        "drop it slowly",
        "drop it faster",
        "drop it at 20%",
        "drop it at 20 cm/s",
        "drop it at 0.2 m/s",
        "drop it at two hundred mm/s",
        "drop it at 200 mm/s but slowly",
    ],
)
def test_invalid_or_ambiguous_speed_is_not_silently_ignored(prompt):
    with pytest.raises(VisionMotionError):
        requested_cartesian_velocity(prompt, 25, 325)


def test_default_does_not_exceed_driver_limit():
    with pytest.raises(VisionMotionError, match="range"):
        requested_cartesian_velocity("drop it", 25, 10)


def test_omitting_speed_does_not_reuse_previous_requested_speed():
    assert requested_cartesian_velocity("drop it at 200 mm/s", 25, 325) == 200
    assert requested_cartesian_velocity("drop it", 25, 325) == 25
