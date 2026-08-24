#!/usr/bin/env python3
# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 connection bringup and robot state publisher."""

import math
import threading
from typing import Any, Optional, Sequence

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from fanucpy_ros2_interfaces.action import JogCartesian, RunProgram
from fanucpy_ros2_interfaces.msg import CartesianState, DriverStatus
from fanucpy_ros2_interfaces.srv import (
    GetDigitalOutput,
    GetNumericRegister,
    GetPower,
    SetDigitalOutput,
    SetNumericRegister,
)
from fanucpy_ros2_trajectory_controller.controller import (
    FanucpyFollowJointTrajectoryServer,
    TrajectoryControllerConfig,
)

from .conversions import fanuc_wpr_degrees_to_quaternion
from .motion import validate_cartesian_offset, validate_cartesian_velocity
from .transport import FanucpyDependencyError, FanucpyTransport, RobotStateSnapshot


class FanucpyDriverNode(Node):
    """Connect to one FANUC controller and publish coherent state samples."""

    def __init__(self) -> None:
        super().__init__("fanucpy_driver")

        self.declare_parameter("robot_ip", "192.0.2.10")
        self.declare_parameter("robot_port", 18735)
        self.declare_parameter("robot_model", "Fanuc")
        self.declare_parameter("socket_timeout_sec", 5.0)
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("state_poll_rate_hz", 5.0)
        self.declare_parameter("frame_id", "fanuc_world")
        self.declare_parameter("ee_do_type", "RDO")
        self.declare_parameter("ee_do_num", 7)
        self.declare_parameter("enable_controller_writes", False)
        self.declare_parameter("enable_program_execution", False)
        self.declare_parameter("allowed_tp_programs", [""])
        self.declare_parameter("recycle_connection_after_program", True)
        self.declare_parameter("program_reconnect_timeout_sec", 15.0)
        self.declare_parameter("program_state_probe_timeout_sec", 5.0)
        self.declare_parameter("enable_motion_commands", False)
        self.declare_parameter("max_translation_step_mm", 50.0)
        self.declare_parameter("max_rotation_step_deg", 2.0)
        self.declare_parameter("cartesian_velocity_mm_s", 25)
        self.declare_parameter("max_cartesian_velocity_mm_s", 2000)
        self.declare_parameter("cartesian_acceleration_percent", 20)
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter(
            "trajectory_action_name",
            "/fanuc_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("max_trajectory_points", 500)
        self.declare_parameter("max_joint_step_rad", 0.35)
        self.declare_parameter("trajectory_execution_mode", "stop_at_waypoints")
        self.declare_parameter("allow_goal_only_execution", False)
        self.declare_parameter("goal_only_max_joint_delta_rad", 0.35)
        self.declare_parameter("trajectory_start_tolerance_rad", 0.05)
        self.declare_parameter("trajectory_path_tolerance_rad", 0.05)
        self.declare_parameter("trajectory_goal_tolerance_rad", 0.02)
        self.declare_parameter("joint_velocity_percent", 5)
        self.declare_parameter("max_joint_velocity_percent", 10)
        self.declare_parameter("joint_acceleration_percent", 20)
        self.declare_parameter(
            "joint_lower_limits_rad",
            [-3.14, -1.57, -3.14, -3.31, -3.31, -6.28],
        )
        self.declare_parameter(
            "joint_upper_limits_rad",
            [3.14, 2.79, 4.61, 3.31, 3.31, 6.28],
        )
        self.declare_parameter(
            "joint_velocity_limits_rad_s",
            [3.67, 3.32, 3.67, 6.98, 6.98, 10.47],
        )

        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.robot_port = int(self.get_parameter("robot_port").value)
        self.robot_model = str(self.get_parameter("robot_model").value)
        self.socket_timeout_sec = float(self.get_parameter("socket_timeout_sec").value)
        self.reconnect_delay_sec = float(self.get_parameter("reconnect_delay_sec").value)
        self.state_poll_rate_hz = float(self.get_parameter("state_poll_rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.ee_do_type = str(self.get_parameter("ee_do_type").value).upper()
        self.ee_do_num = int(self.get_parameter("ee_do_num").value)
        self.enable_controller_writes = bool(
            self.get_parameter("enable_controller_writes").value
        )
        self.enable_program_execution = bool(
            self.get_parameter("enable_program_execution").value
        )
        self.allowed_tp_programs = tuple(
            dict.fromkeys(
                FanucpyTransport.normalize_program_name(name)
                for name in self.get_parameter("allowed_tp_programs").value
                if str(name).strip()
            )
        )
        self.recycle_connection_after_program = bool(
            self.get_parameter("recycle_connection_after_program").value
        )
        self.program_reconnect_timeout_sec = float(
            self.get_parameter("program_reconnect_timeout_sec").value
        )
        self.program_state_probe_timeout_sec = float(
            self.get_parameter("program_state_probe_timeout_sec").value
        )
        self.enable_motion_commands = bool(
            self.get_parameter("enable_motion_commands").value
        )
        self.max_translation_step_mm = float(
            self.get_parameter("max_translation_step_mm").value
        )
        self.max_rotation_step_deg = float(
            self.get_parameter("max_rotation_step_deg").value
        )
        self.cartesian_velocity_mm_s = int(
            self.get_parameter("cartesian_velocity_mm_s").value
        )
        self.max_cartesian_velocity_mm_s = int(
            self.get_parameter("max_cartesian_velocity_mm_s").value
        )
        self.cartesian_acceleration_percent = int(
            self.get_parameter("cartesian_acceleration_percent").value
        )
        self.joint_names = [str(name) for name in self.get_parameter("joint_names").value]
        self.trajectory_action_name = str(
            self.get_parameter("trajectory_action_name").value
        )
        self.max_trajectory_points = int(
            self.get_parameter("max_trajectory_points").value
        )
        self.max_joint_step_rad = float(
            self.get_parameter("max_joint_step_rad").value
        )
        self.trajectory_execution_mode = str(
            self.get_parameter("trajectory_execution_mode").value
        )
        self.allow_goal_only_execution = bool(
            self.get_parameter("allow_goal_only_execution").value
        )
        self.goal_only_max_joint_delta_rad = float(
            self.get_parameter("goal_only_max_joint_delta_rad").value
        )
        self.trajectory_start_tolerance_rad = float(
            self.get_parameter("trajectory_start_tolerance_rad").value
        )
        self.trajectory_path_tolerance_rad = float(
            self.get_parameter("trajectory_path_tolerance_rad").value
        )
        self.trajectory_goal_tolerance_rad = float(
            self.get_parameter("trajectory_goal_tolerance_rad").value
        )
        self.joint_velocity_percent = int(
            self.get_parameter("joint_velocity_percent").value
        )
        self.max_joint_velocity_percent = int(
            self.get_parameter("max_joint_velocity_percent").value
        )
        self.joint_acceleration_percent = int(
            self.get_parameter("joint_acceleration_percent").value
        )
        self.joint_lower_limits_rad = tuple(
            float(value)
            for value in self.get_parameter("joint_lower_limits_rad").value
        )
        self.joint_upper_limits_rad = tuple(
            float(value)
            for value in self.get_parameter("joint_upper_limits_rad").value
        )
        self.joint_velocity_limits_rad_s = tuple(
            float(value)
            for value in self.get_parameter("joint_velocity_limits_rad_s").value
        )

        self._validate_parameters()

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_publisher = self.create_publisher(
            DriverStatus,
            "driver_status",
            status_qos,
        )
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.cartesian_state_publisher = self.create_publisher(
            CartesianState,
            "cartesian_state",
            10,
        )
        self.cartesian_pose_publisher = self.create_publisher(
            PoseStamped,
            "cartesian_pose",
            10,
        )
        self._get_numeric_register_service = self.create_service(
            GetNumericRegister,
            "get_numeric_register",
            self._get_numeric_register_callback,
        )
        self._set_numeric_register_service = self.create_service(
            SetNumericRegister,
            "set_numeric_register",
            self._set_numeric_register_callback,
        )
        self._get_digital_output_service = self.create_service(
            GetDigitalOutput,
            "get_digital_output",
            self._get_digital_output_callback,
        )
        self._set_digital_output_service = self.create_service(
            SetDigitalOutput,
            "set_digital_output",
            self._set_digital_output_callback,
        )
        self._set_gripper_service = self.create_service(
            SetBool,
            "set_gripper",
            self._set_gripper_callback,
        )
        self._get_power_service = self.create_service(
            GetPower,
            "get_power",
            self._get_power_callback,
        )

        self._transport: Optional[FanucpyTransport] = None
        self._motion_lock = threading.Lock()
        self._motion_goal_reserved = False
        self._state_lock = threading.Lock()
        self._latest_joint_positions_rad: Optional[tuple[float, ...]] = None
        self._jog_action_server = ActionServer(
            self,
            JogCartesian,
            "jog_cartesian",
            execute_callback=self._execute_cartesian_jog,
            goal_callback=self._cartesian_jog_goal_callback,
            cancel_callback=self._cartesian_jog_cancel_callback,
        )
        self._program_action_server = ActionServer(
            self,
            RunProgram,
            "run_program",
            execute_callback=self._execute_program,
            goal_callback=self._program_goal_callback,
            cancel_callback=self._program_cancel_callback,
        )
        self._trajectory_controller = FanucpyFollowJointTrajectoryServer(
            node=self,
            config=TrajectoryControllerConfig(
                action_name=self.trajectory_action_name,
                joint_names=tuple(self.joint_names),
                lower_position_limits_rad=self.joint_lower_limits_rad,
                upper_position_limits_rad=self.joint_upper_limits_rad,
                velocity_limits_rad_s=self.joint_velocity_limits_rad_s,
                max_points=self.max_trajectory_points,
                max_joint_step_rad=self.max_joint_step_rad,
                start_tolerance_rad=self.trajectory_start_tolerance_rad,
                path_tolerance_rad=self.trajectory_path_tolerance_rad,
                goal_tolerance_rad=self.trajectory_goal_tolerance_rad,
                default_velocity_percent=self.joint_velocity_percent,
                maximum_velocity_percent=self.max_joint_velocity_percent,
                execution_mode=self.trajectory_execution_mode,
                allow_goal_only_execution=self.allow_goal_only_execution,
                goal_only_max_joint_delta_rad=(
                    self.goal_only_max_joint_delta_rad
                ),
            ),
            can_execute=self._trajectory_can_execute,
            reserve_motion=self._try_reserve_motion,
            release_motion=self._release_motion,
            current_positions=self._current_joint_positions,
            execute_waypoint=self._execute_joint_waypoint,
        )

        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._connection_worker,
            name="fanucpy-connection-worker",
            daemon=True,
        )

        self._publish_status(DriverStatus.DISCONNECTED, "Driver starting")
        if self.enable_motion_commands:
            self.get_logger().warning(
                "Robot motion commands are ENABLED; keep the pendant and "
                "emergency stop available"
            )
        else:
            self.get_logger().info(
                "Robot motion commands are disabled. Set "
                "enable_motion_commands:=true only for supervised testing."
            )
        if self.trajectory_execution_mode == "goal_only":
            if self.allow_goal_only_execution:
                self.get_logger().warning(
                    "GOAL-ONLY trajectory execution is ENABLED: intermediate "
                    "MoveIt waypoints will be skipped and collision-path "
                    "guarantees do not apply to the direct FANUC joint move"
                )
            else:
                self.get_logger().warning(
                    "goal_only mode is selected but remains blocked because "
                    "allow_goal_only_execution is false"
                )
        if self.enable_controller_writes:
            self.get_logger().warning(
                "Controller register, digital-output, and gripper writes are "
                "ENABLED"
            )
        if self.enable_program_execution:
            self.get_logger().warning(
                "TP program execution gate is ENABLED; allowlisted programs: "
                f"{list(self.allowed_tp_programs)}"
            )
            if self.recycle_connection_after_program:
                self.get_logger().info(
                    "Post-program MAPPDK socket recycling is enabled"
                )
        self._worker.start()

    def _validate_parameters(self) -> None:
        if not self.robot_ip.strip():
            raise ValueError("robot_ip must not be empty")
        if not 1 <= self.robot_port <= 65535:
            raise ValueError("robot_port must be in the range 1..65535")
        if self.socket_timeout_sec <= 0.0:
            raise ValueError("socket_timeout_sec must be greater than zero")
        if self.reconnect_delay_sec < 0.1:
            raise ValueError("reconnect_delay_sec must be at least 0.1")
        if not 0.1 <= self.state_poll_rate_hz <= 100.0:
            raise ValueError("state_poll_rate_hz must be in the range 0.1..100")
        if len(self.joint_names) != 6 or len(set(self.joint_names)) != 6:
            raise ValueError("joint_names must contain six unique names")
        self.ee_do_type, self.ee_do_num = (
            FanucpyTransport.validate_digital_output(
                self.ee_do_type,
                self.ee_do_num,
            )
        )
        if self.enable_program_execution and not self.allowed_tp_programs:
            raise ValueError(
                "enable_program_execution requires at least one "
                "allowed_tp_programs entry"
            )
        if self.program_reconnect_timeout_sec <= 0.0:
            raise ValueError(
                "program_reconnect_timeout_sec must be greater than zero"
            )
        if self.program_state_probe_timeout_sec <= 0.0:
            raise ValueError(
                "program_state_probe_timeout_sec must be greater than zero"
            )
        if (
            self.program_state_probe_timeout_sec
            > self.program_reconnect_timeout_sec
        ):
            raise ValueError(
                "program_state_probe_timeout_sec must not exceed "
                "program_reconnect_timeout_sec"
            )
        if not 0.1 <= self.max_translation_step_mm <= 100.0:
            raise ValueError("max_translation_step_mm must be in the range 0.1..100")
        if not 0.1 <= self.max_rotation_step_deg <= 30.0:
            raise ValueError("max_rotation_step_deg must be in the range 0.1..30")
        if not 1 <= self.max_cartesian_velocity_mm_s <= 9999:
            raise ValueError(
                "max_cartesian_velocity_mm_s must be in the range 1..9999"
            )
        validate_cartesian_velocity(
            self.cartesian_velocity_mm_s,
            self.cartesian_velocity_mm_s,
            self.max_cartesian_velocity_mm_s,
        )
        if not 1 <= self.cartesian_acceleration_percent <= 100:
            raise ValueError(
                "cartesian_acceleration_percent must be in the range 1..100"
            )
        if not self.trajectory_action_name.strip():
            raise ValueError("trajectory_action_name must not be empty")
        if not 1 <= self.max_trajectory_points <= 10000:
            raise ValueError("max_trajectory_points must be in the range 1..10000")
        if not 0.01 <= self.max_joint_step_rad <= math.pi:
            raise ValueError("max_joint_step_rad must be in the range 0.01..pi")
        if self.trajectory_execution_mode not in (
            "stop_at_waypoints",
            "goal_only",
        ):
            raise ValueError(
                "trajectory_execution_mode must be stop_at_waypoints or goal_only"
            )
        if not 0.01 <= self.goal_only_max_joint_delta_rad <= math.pi:
            raise ValueError(
                "goal_only_max_joint_delta_rad must be in the range 0.01..pi"
            )
        for name, tolerance in (
            ("trajectory_start_tolerance_rad", self.trajectory_start_tolerance_rad),
            ("trajectory_path_tolerance_rad", self.trajectory_path_tolerance_rad),
            ("trajectory_goal_tolerance_rad", self.trajectory_goal_tolerance_rad),
        ):
            if not 0.001 <= tolerance <= 0.5:
                raise ValueError(f"{name} must be in the range 0.001..0.5")
        if not (
            1
            <= self.joint_velocity_percent
            <= self.max_joint_velocity_percent
            <= 100
        ):
            raise ValueError(
                "joint velocity percentages must satisfy 1 <= default <= max <= 100"
            )
        if not 1 <= self.joint_acceleration_percent <= 100:
            raise ValueError("joint_acceleration_percent must be in the range 1..100")
        if not (
            len(self.joint_lower_limits_rad)
            == len(self.joint_upper_limits_rad)
            == len(self.joint_velocity_limits_rad_s)
            == len(self.joint_names)
        ):
            raise ValueError("Joint position and velocity limits must match joint_names")
        if not all(
            math.isfinite(lower)
            and math.isfinite(upper)
            and lower < upper
            and math.isfinite(velocity)
            and velocity > 0.0
            for lower, upper, velocity in zip(
                self.joint_lower_limits_rad,
                self.joint_upper_limits_rad,
                self.joint_velocity_limits_rad_s,
            )
        ):
            raise ValueError("Joint position or velocity limits are invalid")

    def _trajectory_can_execute(self) -> tuple[bool, str]:
        if not self.enable_motion_commands:
            return False, "motion commands are disabled"
        if (
            self.trajectory_execution_mode == "goal_only"
            and not self.allow_goal_only_execution
        ):
            return False, (
                "goal_only execution requires allow_goal_only_execution=true"
            )
        transport = self._transport
        if transport is None or not transport.connected:
            return False, "robot is not connected"
        return True, "ready"

    def _try_reserve_motion(self) -> bool:
        with self._motion_lock:
            if self._motion_goal_reserved:
                return False
            self._motion_goal_reserved = True
            return True

    def _release_motion(self) -> None:
        with self._motion_lock:
            self._motion_goal_reserved = False

    def _current_joint_positions(self) -> Optional[tuple[float, ...]]:
        with self._state_lock:
            if self._latest_joint_positions_rad is None:
                return None
            return tuple(self._latest_joint_positions_rad)

    def _connected_transport(self) -> FanucpyTransport:
        transport = self._transport
        if transport is None or not transport.connected:
            raise RuntimeError("Robot is not connected")
        return transport

    def _controller_write_ready(self) -> tuple[bool, str]:
        if not self.enable_controller_writes:
            return False, "controller writes are disabled"
        transport = self._transport
        if transport is None or not transport.connected:
            return False, "robot is not connected"
        return True, "ready"

    @staticmethod
    def _digital_output_type_name(output_type: int, rdo: int, dout: int) -> str:
        if int(output_type) == int(rdo):
            return "RDO"
        if int(output_type) == int(dout):
            return "DOUT"
        raise ValueError("output_type must be RDO (1) or DOUT (2)")

    def _get_numeric_register_callback(
        self,
        request: GetNumericRegister.Request,
        response: GetNumericRegister.Response,
    ) -> GetNumericRegister.Response:
        try:
            value = self._connected_transport().get_numeric_register(
                request.register_number
            )
            response.success = True
            if isinstance(value, int):
                response.value_type = GetNumericRegister.Response.INTEGER
                response.integer_value = value
                response.float_value = float(value)
            else:
                response.value_type = GetNumericRegister.Response.FLOAT
                response.float_value = float(value)
            response.message = "Numeric register read successfully"
        except Exception as exc:
            response.success = False
            response.message = f"Numeric register read failed: {exc}"
        return response

    def _set_numeric_register_callback(
        self,
        request: SetNumericRegister.Request,
        response: SetNumericRegister.Response,
    ) -> SetNumericRegister.Response:
        allowed, reason = self._controller_write_ready()
        if not allowed:
            response.success = False
            response.message = reason
            return response
        if not self._try_reserve_motion():
            response.success = False
            response.message = "another controller command is executing"
            return response

        try:
            if request.value_type == SetNumericRegister.Request.INTEGER:
                value: int | float = int(request.integer_value)
            elif request.value_type == SetNumericRegister.Request.FLOAT:
                value = float(request.float_value)
            else:
                raise ValueError("value_type must be INTEGER (1) or FLOAT (2)")
            code, message = self._connected_transport().set_numeric_register(
                request.register_number,
                value,
            )
            response.success = int(code) == 0
            response.message = str(message)
        except Exception as exc:
            response.success = False
            response.message = f"Numeric register write failed: {exc}"
        finally:
            self._release_motion()
        return response

    def _get_digital_output_callback(
        self,
        request: GetDigitalOutput.Request,
        response: GetDigitalOutput.Response,
    ) -> GetDigitalOutput.Response:
        try:
            output_type = self._digital_output_type_name(
                request.output_type,
                GetDigitalOutput.Request.RDO,
                GetDigitalOutput.Request.DOUT,
            )
            response.value = self._connected_transport().get_digital_output(
                output_type,
                request.output_number,
            )
            response.success = True
            response.message = f"{output_type} read successfully"
        except Exception as exc:
            response.success = False
            response.message = f"Digital output read failed: {exc}"
        return response

    def _set_digital_output_callback(
        self,
        request: SetDigitalOutput.Request,
        response: SetDigitalOutput.Response,
    ) -> SetDigitalOutput.Response:
        allowed, reason = self._controller_write_ready()
        if not allowed:
            response.success = False
            response.message = reason
            return response
        if not self._try_reserve_motion():
            response.success = False
            response.message = "another controller command is executing"
            return response

        try:
            output_type = self._digital_output_type_name(
                request.output_type,
                SetDigitalOutput.Request.RDO,
                SetDigitalOutput.Request.DOUT,
            )
            code, message = self._connected_transport().set_digital_output(
                output_type,
                request.output_number,
                request.value,
            )
            response.success = int(code) == 0
            response.message = str(message)
        except Exception as exc:
            response.success = False
            response.message = f"Digital output write failed: {exc}"
        finally:
            self._release_motion()
        return response

    def _set_gripper_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        allowed, reason = self._controller_write_ready()
        if not allowed:
            response.success = False
            response.message = reason
            return response
        if not self._try_reserve_motion():
            response.success = False
            response.message = "another controller command is executing"
            return response

        try:
            code, message = self._connected_transport().set_gripper(request.data)
            response.success = int(code) == 0
            response.message = str(message)
        except Exception as exc:
            response.success = False
            response.message = f"Gripper write failed: {exc}"
        finally:
            self._release_motion()
        return response

    def _get_power_callback(
        self,
        _request: GetPower.Request,
        response: GetPower.Response,
    ) -> GetPower.Response:
        try:
            response.watts = (
                self._connected_transport().get_instantaneous_power_w()
            )
            response.success = True
            response.message = "Instantaneous power read successfully"
        except Exception as exc:
            response.success = False
            response.message = f"Power read failed: {exc}"
        return response

    def _program_goal_callback(
        self,
        goal_request: RunProgram.Goal,
    ) -> GoalResponse:
        try:
            program_name = FanucpyTransport.normalize_program_name(
                goal_request.program_name
            )
        except ValueError as exc:
            self.get_logger().warning(f"Rejected TP program: {exc}")
            return GoalResponse.REJECT
        if not self.enable_motion_commands:
            reason = "motion commands are disabled"
        elif not self.enable_program_execution:
            reason = "TP program execution is disabled"
        elif program_name not in self.allowed_tp_programs:
            reason = f"TP program {program_name} is not allowlisted"
        else:
            transport = self._transport
            reason = (
                "robot is not connected"
                if transport is None or not transport.connected
                else ""
            )
        if reason:
            self.get_logger().warning(f"Rejected TP program: {reason}")
            return GoalResponse.REJECT
        if not self._try_reserve_motion():
            self.get_logger().warning(
                "Rejected TP program: another controller command is executing"
            )
            return GoalResponse.REJECT
        self.get_logger().warning(
            f"Accepted allowlisted TP program {program_name} for execution"
        )
        return GoalResponse.ACCEPT

    def _program_cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self.get_logger().warning(
            "TP program cancellation rejected: MAPPDK has no dependable "
            "remote abort; use teach-pendant HOLD or emergency stop"
        )
        return CancelResponse.REJECT

    def _execute_program(self, goal_handle: Any) -> RunProgram.Result:
        result = RunProgram.Result()
        try:
            program_name = FanucpyTransport.normalize_program_name(
                goal_handle.request.program_name
            )
            if (
                not self.enable_motion_commands
                or not self.enable_program_execution
                or program_name not in self.allowed_tp_programs
            ):
                raise RuntimeError("TP program execution gate changed")

            feedback = RunProgram.Feedback()
            feedback.state = f"Executing TP program {program_name}"
            goal_handle.publish_feedback(feedback)
            transport = self._connected_transport()
            if self.recycle_connection_after_program:
                program_result = transport.call_program_with_session_recycle(
                    program_name,
                    reconnect_timeout_sec=self.program_reconnect_timeout_sec,
                    reconnect_retry_sec=self.reconnect_delay_sec,
                    state_probe_timeout_sec=(
                        self.program_state_probe_timeout_sec
                    ),
                )
                code = program_result.response_code
                message = (
                    f"{program_result.response_message}; "
                    f"{program_result.recovery_message}"
                )
                if program_result.recovered_state is not None:
                    self._publish_snapshot(program_result.recovered_state)
                if code == 0 and not program_result.connection_ready:
                    self.get_logger().error(program_result.recovery_message)
            else:
                code, message = transport.call_program(program_name)

            result.response_code = int(code)
            result.success = int(code) == 0
            result.message = str(message)
            if result.success:
                goal_handle.succeed()
                self.get_logger().info(
                    f"TP program {program_name} completed: {message}"
                )
            else:
                goal_handle.abort()
                self.get_logger().error(
                    f"TP program {program_name} failed: {message}"
                )
        except Exception as exc:
            result.success = False
            result.response_code = 1
            result.message = f"TP program execution failed: {exc}"
            goal_handle.abort()
            self.get_logger().error(result.message)
        finally:
            self._release_motion()
        return result

    def _cartesian_jog_goal_callback(self, goal_request: Any) -> GoalResponse:
        if not self.enable_motion_commands:
            self.get_logger().warning(
                "Rejected Cartesian jog: motion commands are disabled"
            )
            return GoalResponse.REJECT

        transport = self._transport
        if transport is None or not transport.connected:
            self.get_logger().warning(
                "Rejected Cartesian jog: robot is not connected"
            )
            return GoalResponse.REJECT

        requested_frame = goal_request.header.frame_id.strip()
        if requested_frame and requested_frame != self.frame_id:
            self.get_logger().warning(
                f"Rejected Cartesian jog: frame '{requested_frame}' does not "
                f"match configured frame '{self.frame_id}'"
            )
            return GoalResponse.REJECT

        try:
            validate_cartesian_offset(
                self._jog_values(goal_request),
                self.max_translation_step_mm,
                self.max_rotation_step_deg,
            )
            validate_cartesian_velocity(
                goal_request.velocity_mm_s,
                self.cartesian_velocity_mm_s,
                self.max_cartesian_velocity_mm_s,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Rejected Cartesian jog: {exc}")
            return GoalResponse.REJECT

        if not self._try_reserve_motion():
            self.get_logger().warning(
                "Rejected Cartesian jog: another motion is executing"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cartesian_jog_cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self.get_logger().warning(
            "Jog cancellation rejected: fanucpy/MAPPDK does not expose a "
            "controller motion-abort command; use the teach pendant HOLD or E-stop"
        )
        return CancelResponse.REJECT

    @staticmethod
    def _jog_values(goal: Any) -> tuple[float, float, float, float, float, float]:
        return (
            goal.delta_x_mm,
            goal.delta_y_mm,
            goal.delta_z_mm,
            goal.delta_w_deg,
            goal.delta_p_deg,
            goal.delta_r_deg,
        )

    def _execute_cartesian_jog(self, goal_handle: Any) -> JogCartesian.Result:
        result = JogCartesian.Result()
        feedback = JogCartesian.Feedback()
        feedback.state = "Executing bounded Cartesian jog"
        goal_handle.publish_feedback(feedback)

        try:
            offset = validate_cartesian_offset(
                self._jog_values(goal_handle.request),
                self.max_translation_step_mm,
                self.max_rotation_step_deg,
            )
            velocity_mm_s = validate_cartesian_velocity(
                goal_handle.request.velocity_mm_s,
                self.cartesian_velocity_mm_s,
                self.max_cartesian_velocity_mm_s,
            )
            transport = self._transport
            if transport is None or not transport.connected:
                raise RuntimeError("Robot disconnected before motion execution")

            motion = transport.jog_cartesian(
                offset,
                velocity_mm_s=velocity_mm_s,
                acceleration_percent=self.cartesian_acceleration_percent,
            )
            target = motion.target_mm_deg
            (
                result.target_x_mm,
                result.target_y_mm,
                result.target_z_mm,
                result.target_w_deg,
                result.target_p_deg,
                result.target_r_deg,
            ) = target

            if motion.response_code != 0:
                raise RuntimeError(motion.response_message)

            result.success = True
            result.message = motion.response_message or "Cartesian jog completed"
            goal_handle.succeed()
            self.get_logger().info(
                "Cartesian jog completed; target="
                f"[{', '.join(f'{value:.3f}' for value in target)}]; "
                f"velocity={velocity_mm_s} mm/s"
            )
        except Exception as exc:
            result.success = False
            result.message = f"Cartesian jog failed: {exc}"
            goal_handle.abort()
            self.get_logger().error(result.message)
        finally:
            self._release_motion()

        return result

    def _publish_status(self, state: int, message: str) -> None:
        status = DriverStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = state
        status.message = str(message)
        status.robot_model = self.robot_model
        status.host = self.robot_ip
        status.port = self.robot_port
        status.motion_commands_enabled = self.enable_motion_commands
        status.max_translation_step_mm = self.max_translation_step_mm
        status.max_rotation_step_deg = self.max_rotation_step_deg
        status.default_cartesian_velocity_mm_s = self.cartesian_velocity_mm_s
        status.max_cartesian_velocity_mm_s = self.max_cartesian_velocity_mm_s
        status.default_joint_velocity_percent = self.joint_velocity_percent
        status.max_joint_velocity_percent = self.max_joint_velocity_percent
        status.trajectory_execution_mode = self.trajectory_execution_mode
        status.goal_only_execution_enabled = (
            self.trajectory_execution_mode == "goal_only"
            and self.allow_goal_only_execution
        )
        status.goal_only_max_joint_delta_rad = (
            self.goal_only_max_joint_delta_rad
        )
        status.trajectory_action_name = self.trajectory_action_name
        status.controller_writes_enabled = self.enable_controller_writes
        status.program_execution_enabled = (
            self.enable_motion_commands
            and self.enable_program_execution
            and bool(self.allowed_tp_programs)
        )
        status.allowed_tp_programs = list(self.allowed_tp_programs)
        status.gripper_output_type = self.ee_do_type
        status.gripper_output_number = self.ee_do_num
        self.status_publisher.publish(status)

    def _connection_worker(self) -> None:
        transport = FanucpyTransport(
            robot_model=self.robot_model,
            host=self.robot_ip,
            port=self.robot_port,
            socket_timeout_sec=self.socket_timeout_sec,
            ee_do_type=self.ee_do_type,
            ee_do_num=self.ee_do_num,
        )
        self._transport = transport
        poll_period = 1.0 / self.state_poll_rate_hz

        try:
            while not self._stop_event.is_set() and rclpy.ok():
                self._publish_status(
                    DriverStatus.CONNECTING,
                    f"Connecting to {self.robot_ip}:{self.robot_port}",
                )

                try:
                    transport.connect()
                except FanucpyDependencyError as exc:
                    self._publish_status(DriverStatus.ERROR, str(exc))
                    self.get_logger().fatal(str(exc))
                    return
                except Exception as exc:
                    message = f"Connection failed: {exc}"
                    self._publish_status(DriverStatus.ERROR, message)
                    self.get_logger().error(message)
                    transport.disconnect()
                    self._stop_event.wait(self.reconnect_delay_sec)
                    continue

                connected_message = (
                    f"Connected to FANUC controller at "
                    f"{self.robot_ip}:{self.robot_port}"
                )
                self._publish_status(DriverStatus.CONNECTED, connected_message)
                self.get_logger().info(connected_message)

                while not self._stop_event.is_set() and rclpy.ok():
                    try:
                        snapshot = transport.read_state()
                        self._publish_snapshot(snapshot)
                    except Exception as exc:
                        message = f"Robot state read failed: {exc}"
                        self._publish_status(DriverStatus.ERROR, message)
                        self.get_logger().error(message)
                        break

                    self._stop_event.wait(poll_period)

                transport.disconnect()
                if not self._stop_event.is_set():
                    self._publish_status(
                        DriverStatus.DISCONNECTED,
                        "Connection lost; waiting to reconnect",
                    )
                    self._stop_event.wait(self.reconnect_delay_sec)
        finally:
            transport.disconnect()
            if rclpy.ok():
                self._publish_status(DriverStatus.DISCONNECTED, "Driver stopped")

    def _publish_snapshot(self, snapshot: RobotStateSnapshot) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_joint_positions(
            tuple(math.radians(value) for value in snapshot.joints_deg),
            stamp=stamp,
        )

        x_mm, y_mm, z_mm, w_deg, p_deg, r_deg = snapshot.cartesian_mm_deg

        cartesian_message = CartesianState()
        cartesian_message.header.stamp = stamp
        cartesian_message.header.frame_id = self.frame_id
        cartesian_message.x_mm = x_mm
        cartesian_message.y_mm = y_mm
        cartesian_message.z_mm = z_mm
        cartesian_message.w_deg = w_deg
        cartesian_message.p_deg = p_deg
        cartesian_message.r_deg = r_deg
        self.cartesian_state_publisher.publish(cartesian_message)

        qx, qy, qz, qw = fanuc_wpr_degrees_to_quaternion(w_deg, p_deg, r_deg)
        pose_message = PoseStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = self.frame_id
        pose_message.pose.position.x = x_mm / 1000.0
        pose_message.pose.position.y = y_mm / 1000.0
        pose_message.pose.position.z = z_mm / 1000.0
        pose_message.pose.orientation.x = qx
        pose_message.pose.orientation.y = qy
        pose_message.pose.orientation.z = qz
        pose_message.pose.orientation.w = qw
        self.cartesian_pose_publisher.publish(pose_message)

    def _publish_joint_positions(
        self,
        positions_rad: Sequence[float],
        stamp: Optional[Any] = None,
    ) -> None:
        positions = tuple(float(value) for value in positions_rad)
        with self._state_lock:
            self._latest_joint_positions_rad = positions

        joint_message = JointState()
        joint_message.header.stamp = stamp or self.get_clock().now().to_msg()
        joint_message.name = self.joint_names
        joint_message.position = list(positions)
        self.joint_publisher.publish(joint_message)

    def _execute_joint_waypoint(
        self,
        positions_rad: Sequence[float],
        velocity_percent: int,
    ) -> tuple[tuple[float, ...], str]:
        transport = self._transport
        if transport is None or not transport.connected:
            raise RuntimeError("Robot disconnected before joint waypoint execution")
        result = transport.move_joint(
            tuple(math.degrees(value) for value in positions_rad),
            velocity_percent=velocity_percent,
            acceleration_percent=self.joint_acceleration_percent,
        )
        if result.response_code != 0:
            raise RuntimeError(result.response_message)
        actual_rad = tuple(math.radians(value) for value in result.actual_deg)
        self._publish_joint_positions(actual_rad)
        return actual_rad, result.response_message or "joint waypoint completed"

    def stop(self) -> None:
        """Request worker shutdown and wait for the socket timeout window."""
        if self._stop_event.is_set():
            return
        self._stop_event.set()
        self._worker.join(timeout=self.socket_timeout_sec + 1.0)
        if self._worker.is_alive():
            self.get_logger().warning(
                "Connection worker did not stop before the shutdown timeout"
            )

    def destroy_node(self) -> None:
        """Destroy the action waitable before destroying normal node entities."""
        self._trajectory_controller.destroy()
        self._jog_action_server.destroy()
        self._program_action_server.destroy()
        super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[FanucpyDriverNode] = None
    try:
        node = FanucpyDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
