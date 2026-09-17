# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import pytest

from fanucpy_ros2_vision_bridge.affine_transform import EyeToHandAffine


MATRIX = (-0.01645279, -1.00536662, -1.00400079, -0.00456943)
PHASE_TWO_TRANSLATION = (-170.43887235, 1309.18504954)


def test_phase_two_active_affine_matches_direct_equation():
    calibration = EyeToHandAffine.from_parameters(
        MATRIX,
        PHASE_TWO_TRANSLATION,
    )
    x_mm, y_mm = calibration.transform(0.20, 0.30, 0.89, "top")
    assert x_mm == pytest.approx(
        MATRIX[0] * 200.0
        + MATRIX[1] * 300.0
        + PHASE_TWO_TRANSLATION[0]
    )
    assert y_mm == pytest.approx(
        MATRIX[2] * 200.0
        + MATRIX[3] * 300.0
        + PHASE_TWO_TRANSLATION[1]
    )


def test_bottom_finish_edge_converts_s_back_to_camera_y():
    calibration = EyeToHandAffine.from_parameters(
        MATRIX,
        PHASE_TWO_TRANSLATION,
    )
    bottom = calibration.transform(0.20, 0.59, 0.89, "bottom")
    top = calibration.transform(0.20, 0.30, 0.89, "top")
    assert bottom == pytest.approx(top)


@pytest.mark.parametrize(
    "matrix, translation",
    [
        ((1.0, 2.0, 3.0), (0.0, 0.0)),
        ((1.0, 2.0, 2.0, 4.0), (0.0, 0.0)),
        ((1.0, 0.0, 0.0, 1.0), (0.0,)),
    ],
)
def test_rejects_invalid_affine_parameters(matrix, translation):
    with pytest.raises(ValueError):
        EyeToHandAffine.from_parameters(matrix, translation)
