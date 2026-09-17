# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Validated 2D conveyor-to-robot affine calibration."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


@dataclass(frozen=True)
class EyeToHandAffine:
    """Map conveyor metric coordinates to a calibrated robot XY frame."""

    matrix: Tuple[float, float, float, float]
    translation_mm: Tuple[float, float]

    @classmethod
    def from_parameters(
        cls,
        matrix: Sequence[float],
        translation_mm: Sequence[float],
    ) -> "EyeToHandAffine":
        """Validate ROS parameter arrays and construct the transform."""
        if len(matrix) != 4:
            raise ValueError("eye_to_hand_matrix must contain four values")
        if len(translation_mm) != 2:
            raise ValueError(
                "eye_to_hand_translation_mm must contain two values"
            )
        values = tuple(float(value) for value in matrix)
        translation = tuple(float(value) for value in translation_mm)
        if not all(math.isfinite(value) for value in values + translation):
            raise ValueError("Eye-to-hand affine values must be finite")
        determinant = values[0] * values[3] - values[1] * values[2]
        if abs(determinant) <= 1e-9:
            raise ValueError("eye_to_hand_matrix must be invertible")
        return cls(
            matrix=values,  # type: ignore[arg-type]
            translation_mm=translation,  # type: ignore[arg-type]
        )

    def transform(
        self,
        center_x_m: float,
        center_s_m: float,
        conveyor_length_m: float,
        finish_edge: str,
    ) -> Tuple[float, float]:
        """Transform conveyor x/s metres into calibrated robot XY mm."""
        inputs = (center_x_m, center_s_m, conveyor_length_m)
        if not all(math.isfinite(value) for value in inputs):
            raise ValueError("Conveyor coordinates must be finite")
        if conveyor_length_m <= 0.0:
            raise ValueError("conveyor_length_m must be positive")
        if finish_edge == "top":
            camera_y_m = center_s_m
        elif finish_edge == "bottom":
            camera_y_m = conveyor_length_m - center_s_m
        else:
            raise ValueError("finish_edge must be 'top' or 'bottom'")
        camera_x_mm = center_x_m * 1000.0
        camera_y_mm = camera_y_m * 1000.0
        a00, a01, a10, a11 = self.matrix
        bx, by = self.translation_mm
        return (
            a00 * camera_x_mm + a01 * camera_y_mm + bx,
            a10 * camera_x_mm + a11 * camera_y_mm + by,
        )
