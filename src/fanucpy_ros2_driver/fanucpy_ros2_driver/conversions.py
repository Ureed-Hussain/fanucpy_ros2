# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Unit and orientation conversions used by the ROS driver."""

import math
from typing import Tuple


Quaternion = Tuple[float, float, float, float]


def fanuc_wpr_degrees_to_quaternion(
    w_deg: float,
    p_deg: float,
    r_deg: float,
) -> Quaternion:
    """
    Convert FANUC W/P/R degrees to a ROS x/y/z/w quaternion.

    The conversion follows the roll-X, pitch-Y, yaw-Z convention used by the
    existing FANUC integration. Controller frame configuration must be
    verified before this pose is used for command generation.
    """
    roll = math.radians(float(w_deg))
    pitch = math.radians(float(p_deg))
    yaw = math.radians(float(r_deg))

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        raise ValueError("Cannot normalize a zero-length quaternion")

    return qx / norm, qy / norm, qz / norm, qw / norm
