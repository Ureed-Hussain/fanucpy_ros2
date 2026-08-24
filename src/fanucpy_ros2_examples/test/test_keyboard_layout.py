# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

from fanucpy_ros2_examples.keyboard_layout import (
    bounded_step,
    offset_for_key,
    translation_preset_for_key,
    velocity_increment,
    velocity_preset_for_key,
)


def test_translation_key_uses_millimetres():
    assert offset_for_key("w", 1.5, 0.5) == (0.0, 1.5, 0.0, 0.0, 0.0, 0.0)


def test_rotation_key_uses_degrees_and_is_case_insensitive():
    assert offset_for_key("L", 1.0, 0.5) == (0.0, 0.0, 0.0, 0.0, 0.0, -0.5)


def test_non_motion_key_has_no_offset():
    assert offset_for_key(" ", 1.0, 0.5) is None


def test_step_adjustment_is_bounded():
    assert bounded_step(1.0, -2.0, 0.5, 5.0) == 0.5
    assert bounded_step(4.5, 2.0, 0.5, 5.0) == 5.0


def test_translation_presets_include_fifty_millimetres():
    assert translation_preset_for_key("1", 50.0) == 1.0
    assert translation_preset_for_key("5", 50.0) == 50.0


def test_translation_preset_respects_a_lower_driver_limit():
    assert translation_preset_for_key("5", 20.0) == 20.0
    assert translation_preset_for_key("x", 50.0) is None


def test_velocity_presets_scale_to_selected_robot_limit():
    assert velocity_preset_for_key("6", 2000) == 200
    assert velocity_preset_for_key("7", 2000) == 500
    assert velocity_preset_for_key("8", 2000) == 1000
    assert velocity_preset_for_key("9", 2000) == 1500
    assert velocity_preset_for_key("0", 2000) == 2000
    assert velocity_preset_for_key("x", 2000) is None


def test_velocity_increment_is_five_percent_with_minimum_one():
    assert velocity_increment(2000) == 100
    assert velocity_increment(10) == 1
