# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Bring up one fanucpy connection and publish robot state."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("fanucpy_ros2_bringup"), "config", "fanuc_m10ia.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Absolute path to the driver parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="fanuc",
            description="ROS namespace for FANUC-specific topics.",
        ),
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
            "state_poll_rate_hz",
            default_value="5.0",
            description="Joint and Cartesian state polling rate.",
        ),
        DeclareLaunchArgument(
            "socket_timeout_sec",
            default_value="5.0",
            description="Timeout for blocking fanucpy socket operations.",
        ),
        DeclareLaunchArgument(
            "reconnect_delay_sec",
            default_value="2.0",
            description="Delay before reconnecting after an error.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="fanuc_world",
            description="Frame label applied to Cartesian state messages.",
        ),
        DeclareLaunchArgument(
            "enable_motion_commands",
            default_value="false",
            description="Allow bounded motion actions when explicitly true.",
        ),
        DeclareLaunchArgument(
            "enable_controller_writes",
            default_value="false",
            choices=["true", "false"],
            description="Allow register, digital-output, and gripper writes.",
        ),
        DeclareLaunchArgument(
            "enable_program_execution",
            default_value="false",
            choices=["true", "false"],
            description="Second gate for allowlisted TP program execution.",
        ),
        DeclareLaunchArgument(
            "allowed_tp_programs",
            default_value='[""]',
            description="YAML list of TP program names permitted by the driver.",
        ),
        DeclareLaunchArgument(
            "recycle_connection_after_program",
            default_value="true",
            choices=["true", "false"],
            description=(
                "Replace and verify the MAPPDK socket after each TP program."
            ),
        ),
        DeclareLaunchArgument(
            "program_reconnect_timeout_sec",
            default_value="15.0",
            description="Maximum post-program MAPPDK recovery time.",
        ),
        DeclareLaunchArgument(
            "program_state_probe_timeout_sec",
            default_value="5.0",
            description="Socket timeout used by the post-program state probe.",
        ),
        DeclareLaunchArgument(
            "max_translation_step_mm",
            default_value="50.0",
            description="Maximum absolute translation in one Cartesian jog.",
        ),
        DeclareLaunchArgument(
            "max_cartesian_velocity_mm_s",
            default_value="2000",
            description=(
                "Robot-specific Cartesian velocity ceiling accepted from a goal."
            ),
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
            description="Preserve all MoveIt points or command only the final target.",
        ),
        DeclareLaunchArgument(
            "allow_goal_only_execution",
            default_value="false",
            choices=["true", "false"],
            description="Second opt-in gate required by goal_only mode.",
        ),
        DeclareLaunchArgument(
            "goal_only_max_joint_delta_rad",
            default_value="0.35",
            description="Maximum current-to-goal distance allowed on each joint.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS log level for the driver node.",
        ),
    ]

    driver = Node(
        package="fanucpy_ros2_driver",
        executable="fanucpy_driver",
        namespace=LaunchConfiguration("namespace"),
        name="fanucpy_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "robot_port": ParameterValue(
                    LaunchConfiguration("robot_port"),
                    value_type=int,
                ),
                "robot_model": LaunchConfiguration("robot_model"),
                "state_poll_rate_hz": ParameterValue(
                    LaunchConfiguration("state_poll_rate_hz"),
                    value_type=float,
                ),
                "socket_timeout_sec": ParameterValue(
                    LaunchConfiguration("socket_timeout_sec"),
                    value_type=float,
                ),
                "reconnect_delay_sec": ParameterValue(
                    LaunchConfiguration("reconnect_delay_sec"),
                    value_type=float,
                ),
                "frame_id": LaunchConfiguration("frame_id"),
                "enable_motion_commands": ParameterValue(
                    LaunchConfiguration("enable_motion_commands"),
                    value_type=bool,
                ),
                "enable_controller_writes": ParameterValue(
                    LaunchConfiguration("enable_controller_writes"),
                    value_type=bool,
                ),
                "enable_program_execution": ParameterValue(
                    LaunchConfiguration("enable_program_execution"),
                    value_type=bool,
                ),
                "allowed_tp_programs": ParameterValue(
                    LaunchConfiguration("allowed_tp_programs"),
                ),
                "recycle_connection_after_program": ParameterValue(
                    LaunchConfiguration("recycle_connection_after_program"),
                    value_type=bool,
                ),
                "program_reconnect_timeout_sec": ParameterValue(
                    LaunchConfiguration("program_reconnect_timeout_sec"),
                    value_type=float,
                ),
                "program_state_probe_timeout_sec": ParameterValue(
                    LaunchConfiguration("program_state_probe_timeout_sec"),
                    value_type=float,
                ),
                "max_translation_step_mm": ParameterValue(
                    LaunchConfiguration("max_translation_step_mm"),
                    value_type=float,
                ),
                "max_cartesian_velocity_mm_s": ParameterValue(
                    LaunchConfiguration("max_cartesian_velocity_mm_s"),
                    value_type=int,
                ),
                "max_joint_velocity_percent": ParameterValue(
                    LaunchConfiguration("max_joint_velocity_percent"),
                    value_type=int,
                ),
                "joint_velocity_percent": ParameterValue(
                    LaunchConfiguration("joint_velocity_percent"),
                    value_type=int,
                ),
                "trajectory_execution_mode": LaunchConfiguration(
                    "trajectory_execution_mode"
                ),
                "allow_goal_only_execution": ParameterValue(
                    LaunchConfiguration("allow_goal_only_execution"),
                    value_type=bool,
                ),
                "goal_only_max_joint_delta_rad": ParameterValue(
                    LaunchConfiguration("goal_only_max_joint_delta_rad"),
                    value_type=float,
                ),
            },
        ],
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(declared_arguments + [driver])
