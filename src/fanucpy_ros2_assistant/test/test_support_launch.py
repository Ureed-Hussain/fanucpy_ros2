# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Check safe defaults and generic external-perception integration."""

from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1] / "launch" / "assistant_support.launch.py"
)


def test_optional_perception_nodes_default_off():
    """Do not unexpectedly start workcell-specific external packages."""
    text = LAUNCH.read_text(encoding="utf-8")
    for argument in ("start_camera", "start_detector"):
        declaration = text.index(f'"{argument}",')
        default = text.index('default_value="false"', declaration)
        assert default > declaration


def test_external_perception_is_configurable_and_conditioned():
    """Allow other installations to provide their own ROS packages."""
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'camera_package = LaunchConfiguration("camera_package")' in text
    assert 'detector_package = LaunchConfiguration("detector_package")' in text
    assert "condition=IfCondition(start_camera)" in text
    assert "condition=IfCondition(start_detector)" in text
    assert 'detector_model_path = LaunchConfiguration(' in text


def test_private_model_is_not_embedded_in_launch_file():
    """Require the operator to supply a local checkpoint explicitly."""
    text = LAUNCH.read_text(encoding="utf-8")
    assert ".pt" not in text
    assert "/home/" not in text
