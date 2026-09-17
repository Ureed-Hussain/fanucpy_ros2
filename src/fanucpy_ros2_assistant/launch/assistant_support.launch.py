# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Start the FANUC driver, vision bridge, and optional perception nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create support bringup with physical gates off by default."""
    robot_ip = LaunchConfiguration("robot_ip")
    frame_id = LaunchConfiguration("frame_id")
    motion_socket_timeout = LaunchConfiguration(
        "motion_socket_timeout_sec"
    )
    motion_gate = LaunchConfiguration("enable_motion_commands")
    absolute_gate = LaunchConfiguration(
        "enable_absolute_cartesian_commands"
    )
    program_gate = LaunchConfiguration("enable_program_execution")
    allowed_programs = LaunchConfiguration("allowed_tp_programs")
    bridge_parameters = LaunchConfiguration("bridge_parameters_file")
    start_camera = LaunchConfiguration("start_camera")
    camera_package = LaunchConfiguration("camera_package")
    camera_launch_file = LaunchConfiguration("camera_launch_file")
    start_detector = LaunchConfiguration("start_detector")
    detector_package = LaunchConfiguration("detector_package")
    detector_executable = LaunchConfiguration("detector_executable")
    detector_model_path = LaunchConfiguration("detector_model_path")
    detector_image_topic = LaunchConfiguration("detector_image_topic")

    bringup_launch = PathJoinSubstitution(
        [
            FindPackageShare("fanucpy_ros2_bringup"),
            "launch",
            "fanucpy_bringup.launch.py",
        ]
    )
    bridge_launch = PathJoinSubstitution(
        [
            FindPackageShare("fanucpy_ros2_vision_bridge"),
            "launch",
            "danger_vision_bridge.launch.py",
        ]
    )
    default_bridge_parameters = PathJoinSubstitution(
        [
            FindPackageShare("fanucpy_ros2_vision_bridge"),
            "config",
            "danger_vision_bridge.yaml",
        ]
    )
    camera_launch = PathJoinSubstitution(
        [
            FindPackageShare(camera_package),
            "launch",
            camera_launch_file,
        ]
    )
    arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.177",
            description="FANUC controller IPv4 address or hostname",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="fanuc_world",
            description="Driver and eye-to-hand command frame",
        ),
        DeclareLaunchArgument(
            "motion_socket_timeout_sec",
            default_value="60.0",
            description="Longer MAPPDK timeout used only during motion",
        ),
        DeclareLaunchArgument(
            "enable_motion_commands",
            default_value="false",
            choices=["true", "false"],
            description="Existing driver-wide motion gate",
        ),
        DeclareLaunchArgument(
            "enable_absolute_cartesian_commands",
            default_value="false",
            choices=["true", "false"],
            description="Existing direct Cartesian target gate",
        ),
        DeclareLaunchArgument(
            "enable_program_execution",
            default_value="false",
            choices=["true", "false"],
            description="Existing TP-program execution gate",
        ),
        DeclareLaunchArgument(
            "allowed_tp_programs",
            default_value='[""]',
            description="Existing exact TP-program allowlist",
        ),
        DeclareLaunchArgument(
            "bridge_parameters_file",
            default_value=default_bridge_parameters,
            description="Vision bridge parameter YAML",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Include an external camera launch file. Its workspace must "
                "already be sourced."
            ),
        ),
        DeclareLaunchArgument(
            "camera_package",
            default_value="flir_launch",
            description="External package containing the camera launch file",
        ),
        DeclareLaunchArgument(
            "camera_launch_file",
            default_value="flir_fast.launch.py",
            description="Launch filename inside camera_package/launch",
        ),
        DeclareLaunchArgument(
            "start_detector",
            default_value="false",
            choices=["true", "false"],
            description=(
                "Start an external detector node. Its workspace and model "
                "dependencies must already be available."
            ),
        ),
        DeclareLaunchArgument(
            "detector_package",
            default_value="danger_vision",
            description="External ROS package containing the detector",
        ),
        DeclareLaunchArgument(
            "detector_executable",
            default_value="danger_model_size_node",
            description="External detector console executable",
        ),
        DeclareLaunchArgument(
            "detector_model_path",
            default_value="",
            description=(
                "Local model checkpoint passed to the external detector; "
                "the model is not included in this repository"
            ),
        ),
        DeclareLaunchArgument(
            "detector_image_topic",
            default_value="/flir_camera/image_raw/compressed",
            description="Compressed image topic consumed by the detector",
        ),
    ]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(start_camera),
    )
    detector = Node(
        package=detector_package,
        executable=detector_executable,
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": ParameterValue(
                    detector_model_path, value_type=str
                ),
                "image_topic": ParameterValue(
                    detector_image_topic, value_type=str
                ),
            }
        ],
        condition=IfCondition(start_detector),
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "robot_ip": robot_ip,
            "frame_id": frame_id,
            "motion_socket_timeout_sec": motion_socket_timeout,
            "enable_motion_commands": motion_gate,
            "enable_absolute_cartesian_commands": absolute_gate,
            "enable_program_execution": program_gate,
            "allowed_tp_programs": allowed_programs,
        }.items(),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={"parameters_file": bridge_parameters}.items(),
    )
    return LaunchDescription(arguments + [camera, detector, driver, bridge])
