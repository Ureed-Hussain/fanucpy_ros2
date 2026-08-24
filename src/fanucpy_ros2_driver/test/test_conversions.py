# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

import math

from fanucpy_ros2_driver.conversions import fanuc_wpr_degrees_to_quaternion


def test_zero_wpr_is_identity_quaternion():
    assert fanuc_wpr_degrees_to_quaternion(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


def test_ninety_degree_roll():
    qx, qy, qz, qw = fanuc_wpr_degrees_to_quaternion(90.0, 0.0, 0.0)
    expected = math.sqrt(0.5)
    assert math.isclose(qx, expected, abs_tol=1e-12)
    assert math.isclose(qy, 0.0, abs_tol=1e-12)
    assert math.isclose(qz, 0.0, abs_tol=1e-12)
    assert math.isclose(qw, expected, abs_tol=1e-12)


def test_quaternion_is_normalized():
    quaternion = fanuc_wpr_degrees_to_quaternion(32.0, -18.0, 147.0)
    norm = math.sqrt(sum(value * value for value in quaternion))
    assert math.isclose(norm, 1.0, abs_tol=1e-12)
