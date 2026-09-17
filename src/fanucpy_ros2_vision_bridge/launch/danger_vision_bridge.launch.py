# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Launch the optional danger_vision detection adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the bridge launch description."""
    package_share = get_package_share_directory(
        "fanucpy_ros2_vision_bridge"
    )
    default_parameters = os.path.join(
        package_share,
        "config",
        "danger_vision_bridge.yaml",
    )
    parameters_file = LaunchConfiguration("parameters_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="Vision bridge ROS parameter file",
            ),
            Node(
                package="fanucpy_ros2_vision_bridge",
                executable="fanucpy_vision_bridge",
                name="fanucpy_vision_bridge",
                output="screen",
                parameters=[parameters_file],
            ),
        ]
    )
