# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Validation helpers for bounded robot motion commands."""

import math
from typing import Sequence, Tuple


CartesianOffset = Tuple[float, float, float, float, float, float]
CartesianTarget = Tuple[float, float, float, float, float, float]


def validate_cartesian_offset(
    values: Sequence[float],
    max_translation_step_mm: float,
    max_rotation_step_deg: float,
) -> CartesianOffset:
    """
    Validate one Cartesian jog offset and return finite float values.

    Limits apply independently to each XYZ and WPR component. Zero-length
    commands are rejected so a key press can never create an ambiguous move.
    """
    if len(values) != 6:
        raise ValueError("A Cartesian jog requires exactly six offset values")
    if max_translation_step_mm <= 0.0 or max_rotation_step_deg <= 0.0:
        raise ValueError("Cartesian jog limits must be greater than zero")

    offset = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in offset):
        raise ValueError("Cartesian jog offsets must be finite")
    if all(value == 0.0 for value in offset):
        raise ValueError("A Cartesian jog offset must not be all zero")

    if any(abs(value) > max_translation_step_mm for value in offset[:3]):
        raise ValueError(
            "Cartesian translation exceeds the configured per-axis step limit"
        )
    if any(abs(value) > max_rotation_step_deg for value in offset[3:]):
        raise ValueError(
            "Cartesian rotation exceeds the configured per-axis step limit"
        )

    return offset  # type: ignore[return-value]


def validate_cartesian_target(
    values: Sequence[float],
    enforce_bounds: bool = False,
    lower_bounds: Sequence[float] = (),
    upper_bounds: Sequence[float] = (),
) -> CartesianTarget:
    """Validate one finite absolute target and optional operator-set bounds."""
    if len(values) != 6:
        raise ValueError("An absolute Cartesian target requires six values")
    target = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in target):
        raise ValueError("Absolute Cartesian target values must be finite")
    if not enforce_bounds:
        return target  # type: ignore[return-value]

    if len(lower_bounds) != 6 or len(upper_bounds) != 6:
        raise ValueError("Absolute Cartesian bounds must contain six values")
    lower = tuple(float(value) for value in lower_bounds)
    upper = tuple(float(value) for value in upper_bounds)
    if not all(
        math.isfinite(low) and math.isfinite(high) and low < high
        for low, high in zip(lower, upper)
    ):
        raise ValueError("Absolute Cartesian bounds are invalid")
    labels = ("X", "Y", "Z", "W", "P", "R")
    for label, value, low, high in zip(labels, target, lower, upper):
        if not low <= value <= high:
            raise ValueError(
                f"Absolute Cartesian {label} target {value:g} is outside "
                f"the configured range [{low:g}, {high:g}]"
            )
    return target  # type: ignore[return-value]


def validate_cartesian_velocity(
    requested_mm_s: int,
    default_mm_s: int,
    maximum_mm_s: int,
) -> int:
    """Resolve a requested velocity and enforce the driver's configured cap."""
    requested = int(requested_mm_s)
    default = int(default_mm_s)
    maximum = int(maximum_mm_s)
    if not 1 <= default <= maximum:
        raise ValueError("Default Cartesian velocity exceeds its configured limits")
    if requested == 0:
        return default
    if not 1 <= requested <= maximum:
        raise ValueError(
            f"Requested Cartesian velocity must be in the range 1..{maximum} mm/s"
        )
    return requested
