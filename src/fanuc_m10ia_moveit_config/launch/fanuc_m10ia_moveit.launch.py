# Copyright 2026 ureed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Start matching MoveIt 2 environments for mock or real FANUC operation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config(use_mock_hardware: bool):
    return (
        MoveItConfigsBuilder(
            "fanuc_m10ia",
            package_name="fanuc_m10ia_moveit_config",
        )
        .robot_description(
            file_path="config/fanuc_m10ia.urdf.xacro",
            mappings={
                "use_mock_hardware": str(use_mock_hardware).lower(),
            },
        )
        .robot_description_semantic(
            file_path="config/fanuc_m10ia.srdf",
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )


def generate_launch_description() -> LaunchDescription:
    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    real_condition = IfCondition(
        PythonExpression(["'", mode, "' == 'real'"])
    )
    mock_condition = IfCondition(
        PythonExpression(["'", mode, "' == 'mock'"])
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            choices=["mock", "real"],
            description="Use ros2_control mock hardware or the real fanucpy driver.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.0.2.10",
            description="FANUC controller address used only in real mode.",
        ),
        DeclareLaunchArgument(
            "robot_port",
            default_value="18735",
            description="MAPPDK port used only in real mode.",
        ),
        DeclareLaunchArgument(
            "enable_motion_commands",
            default_value="false",
            choices=["true", "false"],
            description="Explicit real-robot motion gate; ignored in mock mode.",
        ),
        DeclareLaunchArgument(
            "enable_controller_writes",
            default_value="false",
            choices=["true", "false"],
            description="Enable real-controller register and output writes.",
        ),
        DeclareLaunchArgument(
            "enable_program_execution",
            default_value="false",
            choices=["true", "false"],
            description="Enable allowlisted TP programs in real mode.",
        ),
        DeclareLaunchArgument(
            "allowed_tp_programs",
            default_value='[""]',
            description="YAML list of permitted TP program names.",
        ),
        DeclareLaunchArgument(
            "recycle_connection_after_program",
            default_value="true",
            choices=["true", "false"],
            description="Recycle the MAPPDK socket after each TP program.",
        ),
        DeclareLaunchArgument(
            "program_reconnect_timeout_sec",
            default_value="15.0",
            description="Maximum post-program MAPPDK recovery time.",
        ),
        DeclareLaunchArgument(
            "program_state_probe_timeout_sec",
            default_value="5.0",
            description="Post-program state-probe socket timeout.",
        ),
        DeclareLaunchArgument(
            "state_poll_rate_hz",
            default_value="5.0",
            description="Physical controller state polling rate.",
        ),
        DeclareLaunchArgument(
            "joint_velocity_percent",
            default_value="5",
            description="Default physical FANUC joint velocity percentage.",
        ),
        DeclareLaunchArgument(
            "max_joint_velocity_percent",
            default_value="10",
            description="Maximum physical FANUC joint velocity percentage.",
        ),
        DeclareLaunchArgument(
            "trajectory_execution_mode",
            default_value="stop_at_waypoints",
            choices=["stop_at_waypoints", "goal_only"],
            description=(
                "Real driver mode: preserve every waypoint or command only "
                "the final joint target."
            ),
        ),
        DeclareLaunchArgument(
            "allow_goal_only_execution",
            default_value="false",
            choices=["true", "false"],
            description="Second explicit safety gate required by goal_only.",
        ),
        DeclareLaunchArgument(
            "goal_only_max_joint_delta_rad",
            default_value="0.35",
            description="Maximum direct current-to-goal distance per joint.",
        ),
        DeclareLaunchArgument(
            "base_x",
            default_value="0.0",
            description="World-to-base translation X in metres.",
        ),
        DeclareLaunchArgument(
            "base_y",
            default_value="0.0",
            description="World-to-base translation Y in metres.",
        ),
        DeclareLaunchArgument(
            "base_z",
            default_value="0.0",
            description="World-to-base translation Z in metres.",
        ),
        DeclareLaunchArgument(
            "base_roll",
            default_value="0.0",
            description="World-to-base roll in radians.",
        ),
        DeclareLaunchArgument(
            "base_pitch",
            default_value="0.0",
            description="World-to-base pitch in radians.",
        ),
        DeclareLaunchArgument(
            "base_yaw",
            default_value="0.0",
            description="World-to-base yaw in radians.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz with the MoveIt MotionPlanning panel.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use /clock instead of wall time.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_m10ia_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
            description="RViz configuration file.",
        ),
    ]

    real_moveit_config = _moveit_config(use_mock_hardware=False)
    mock_moveit_config = _moveit_config(use_mock_hardware=True)

    driver_bringup = IncludeLaunchDescription(
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
            "robot_model": "FANUC M-10iA",
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
            "recycle_connection_after_program": LaunchConfiguration(
                "recycle_connection_after_program"
            ),
            "program_reconnect_timeout_sec": LaunchConfiguration(
                "program_reconnect_timeout_sec"
            ),
            "program_state_probe_timeout_sec": LaunchConfiguration(
                "program_state_probe_timeout_sec"
            ),
            "joint_velocity_percent": LaunchConfiguration(
                "joint_velocity_percent"
            ),
            "max_joint_velocity_percent": LaunchConfiguration(
                "max_joint_velocity_percent"
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
        condition=real_condition,
    )

    real_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            real_moveit_config.to_dict(),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "allow_trajectory_execution": True,
            },
        ],
        condition=real_condition,
    )

    mock_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            mock_moveit_config.to_dict(),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "allow_trajectory_execution": True,
            },
        ],
        condition=mock_condition,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="fanuc_robot_state_publisher",
        output="screen",
        parameters=[
            real_moveit_config.robot_description,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="fanuc_world_to_base",
        output="log",
        arguments=[
            "--x",
            LaunchConfiguration("base_x"),
            "--y",
            LaunchConfiguration("base_y"),
            "--z",
            LaunchConfiguration("base_z"),
            "--roll",
            LaunchConfiguration("base_roll"),
            "--pitch",
            LaunchConfiguration("base_pitch"),
            "--yaw",
            LaunchConfiguration("base_yaw"),
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    ros2_controllers = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_m10ia_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    mock_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            mock_moveit_config.robot_description,
            ros2_controllers,
        ],
        condition=mock_condition,
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
        condition=mock_condition,
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fanuc_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
        condition=mock_condition,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fanuc_moveit_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            real_moveit_config.robot_description,
            real_moveit_config.robot_description_semantic,
            real_moveit_config.robot_description_kinematics,
            real_moveit_config.planning_pipelines,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(use_rviz),
    )

    status_messages = [
        LogInfo(
            msg=(
                "FANUC MoveIt mock mode: plans execute through ros2_control "
                "GenericSystem."
            ),
            condition=mock_condition,
        ),
        LogInfo(
            msg=(
                "FANUC MoveIt real mode: physical motion remains blocked unless "
                "enable_motion_commands:=true was explicitly supplied."
            ),
            condition=real_condition,
        ),
    ]

    actions = [
        driver_bringup,
        world_to_base,
        state_publisher,
        mock_control,
        joint_state_spawner,
        arm_controller_spawner,
        real_move_group,
        mock_move_group,
        rviz,
    ]
    return LaunchDescription(declared_arguments + status_messages + actions)
