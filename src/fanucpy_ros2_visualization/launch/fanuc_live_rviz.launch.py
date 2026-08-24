# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Start the FANUC connection, live model transforms, and RViz2 together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.0.2.10",
            description="IPv4 address or hostname of the FANUC controller.",
        ),
        DeclareLaunchArgument(
            "robot_port",
            default_value="18735",
            description="MAPPDK TCP server port.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="FANUC M-10iA",
            description="Robot model label published in driver status.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="fanuc",
            description="ROS namespace for FANUC-specific interfaces.",
        ),
        DeclareLaunchArgument(
            "state_poll_rate_hz",
            default_value="10.0",
            description="Live joint and Cartesian state update rate.",
        ),
        DeclareLaunchArgument(
            "enable_motion_commands",
            default_value="false",
            description="Allow bounded motion actions only when explicitly true.",
        ),
        DeclareLaunchArgument(
            "enable_controller_writes",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "enable_program_execution",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "allowed_tp_programs",
            default_value='[""]',
        ),
        DeclareLaunchArgument(
            "max_translation_step_mm",
            default_value="50.0",
            description="Maximum translation accepted in one Cartesian jog.",
        ),
        DeclareLaunchArgument(
            "max_cartesian_velocity_mm_s",
            default_value="2000",
            description="Robot/site-specific Cartesian velocity ceiling.",
        ),
        DeclareLaunchArgument(
            "joint_velocity_percent",
            default_value="5",
            description="Conservative fallback FANUC joint-speed percentage.",
        ),
        DeclareLaunchArgument(
            "max_joint_velocity_percent",
            default_value="10",
            description="Maximum FANUC joint-speed percentage for trajectories.",
        ),
        DeclareLaunchArgument(
            "trajectory_execution_mode",
            default_value="stop_at_waypoints",
            choices=["stop_at_waypoints", "goal_only"],
        ),
        DeclareLaunchArgument(
            "allow_goal_only_execution",
            default_value="false",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "goal_only_max_joint_delta_rad",
            default_value="0.35",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="moveit_resources_fanuc_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/fanuc.urdf",
        ),
        DeclareLaunchArgument("xacro_args", default_value=""),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanucpy_ros2_bringup"),
                    "launch",
                    "fanucpy_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "robot_port": LaunchConfiguration("robot_port"),
            "robot_model": LaunchConfiguration("robot_model"),
            "namespace": LaunchConfiguration("namespace"),
            "state_poll_rate_hz": LaunchConfiguration("state_poll_rate_hz"),
            "enable_motion_commands": LaunchConfiguration(
                "enable_motion_commands"
            ),
            "enable_controller_writes": LaunchConfiguration(
                "enable_controller_writes"
            ),
            "enable_program_execution": LaunchConfiguration(
                "enable_program_execution"
            ),
            "allowed_tp_programs": LaunchConfiguration(
                "allowed_tp_programs"
            ),
            "max_translation_step_mm": LaunchConfiguration(
                "max_translation_step_mm"
            ),
            "max_cartesian_velocity_mm_s": LaunchConfiguration(
                "max_cartesian_velocity_mm_s"
            ),
            "max_joint_velocity_percent": LaunchConfiguration(
                "max_joint_velocity_percent"
            ),
            "joint_velocity_percent": LaunchConfiguration(
                "joint_velocity_percent"
            ),
            "trajectory_execution_mode": LaunchConfiguration(
                "trajectory_execution_mode"
            ),
            "allow_goal_only_execution": LaunchConfiguration(
                "allow_goal_only_execution"
            ),
            "goal_only_max_joint_delta_rad": LaunchConfiguration(
                "goal_only_max_joint_delta_rad"
            ),
        }.items(),
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanucpy_ros2_visualization"),
                    "launch",
                    "fanuc_visualization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "xacro_args": LaunchConfiguration("xacro_args"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [bringup, visualization])
