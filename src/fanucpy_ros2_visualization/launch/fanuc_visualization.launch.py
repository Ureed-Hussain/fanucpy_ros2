# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Display live FANUC joint states with a selectable robot description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("fanucpy_ros2_visualization"),
            "config",
            "fanuc_live.rviz",
        ]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="moveit_resources_fanuc_description",
            description="Installed ROS 2 package containing the robot xacro.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/fanuc.urdf",
            description="Xacro path relative to the description package share.",
        ),
        DeclareLaunchArgument(
            "xacro_args",
            default_value="",
            description="Optional arguments passed to the selected xacro file.",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="JointState topic produced by the FANUC driver.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz2 configuration file.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 when true.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock when true.",
        ),
    ]

    description_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("description_package")),
            LaunchConfiguration("description_file"),
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                description_path,
                " ",
                LaunchConfiguration("xacro_args"),
            ]
        ),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="fanuc_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            }
        ],
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fanuc_rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                )
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(declared_arguments + [state_publisher, rviz])
