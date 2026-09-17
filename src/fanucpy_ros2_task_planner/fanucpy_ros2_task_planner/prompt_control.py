#!/usr/bin/env python3
# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""CLI client that previews or executes deterministic Cartesian prompts."""

import argparse
import math
import sys
import time
from typing import Any, Optional, Sequence

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from fanucpy_ros2_interfaces.action import JogCartesian, MoveCartesian, RunProgram
from fanucpy_ros2_interfaces.msg import CartesianState, DriverStatus

from .prompt_parser import (
    CartesianJogPlan,
    PromptParseError,
    PromptParser,
    format_plan,
    validate_jog_plan,
)
from .task_plans import (
    CartesianTargetPlan,
    JointTargetPlan,
    TpProgramPlan,
    joint_motion_duration_sec,
    validate_cartesian_target_bounds,
    validate_cartesian_target_plan,
    validate_joint_target_plan,
    validate_tp_program_plan,
)


class FanucpyPromptControl(Node):
    """Own ROS clients and live validation for supported prompt actions."""

    def __init__(self, node_name: str = "fanucpy_prompt_control") -> None:
        super().__init__(node_name)

        self.declare_parameter("action_name", "/fanuc/jog_cartesian")
        self.declare_parameter(
            "absolute_action_name",
            "/fanuc/move_cartesian",
        )
        self.declare_parameter("status_topic", "/fanuc/driver_status")
        self.declare_parameter(
            "cartesian_state_topic",
            "/fanuc/cartesian_state",
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "trajectory_action_name",
            "/fanuc_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("program_action_name", "/fanuc/run_program")
        self.declare_parameter("frame_id", "fanuc_world")
        self.declare_parameter("default_translation_mm", 10.0)
        self.declare_parameter("default_rotation_deg", 1.0)
        self.declare_parameter("max_translation_step_mm", 50.0)
        self.declare_parameter("max_rotation_step_deg", 2.0)
        self.declare_parameter("max_cartesian_velocity_mm_s", 2000)
        self.declare_parameter("status_timeout_sec", 5.0)
        self.declare_parameter("action_server_timeout_sec", 5.0)
        self.declare_parameter("action_result_timeout_sec", 120.0)
        self.declare_parameter("max_cartesian_state_age_sec", 2.0)
        self.declare_parameter("max_joint_state_age_sec", 2.0)
        self.declare_parameter("program_result_timeout_sec", 300.0)
        self.declare_parameter(
            "joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
        )
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
        self.declare_parameter("default_joint_velocity_percent", 5)
        self.declare_parameter("max_joint_velocity_percent", 10)
        self.declare_parameter("max_joint_command_delta_rad", 0.35)

        self.action_name = str(self.get_parameter("action_name").value)
        self.absolute_action_name = str(
            self.get_parameter("absolute_action_name").value
        ).strip()
        self.trajectory_action_name = str(
            self.get_parameter("trajectory_action_name").value
        ).strip()
        self.program_action_name = str(
            self.get_parameter("program_action_name").value
        ).strip()
        self.frame_id = str(self.get_parameter("frame_id").value).strip()
        if not all(
            (
                self.action_name.strip(),
                self.absolute_action_name,
                self.trajectory_action_name,
                self.program_action_name,
                self.frame_id,
            )
        ):
            raise ValueError("Action names and frame_id must not be empty")

        self.parser = PromptParser(
            default_translation_mm=float(
                self.get_parameter("default_translation_mm").value
            ),
            default_rotation_deg=float(
                self.get_parameter("default_rotation_deg").value
            ),
            max_translation_step_mm=float(
                self.get_parameter("max_translation_step_mm").value
            ),
            max_rotation_step_deg=float(
                self.get_parameter("max_rotation_step_deg").value
            ),
            max_cartesian_velocity_mm_s=int(
                self.get_parameter("max_cartesian_velocity_mm_s").value
            ),
        )

        self.status_timeout_sec = float(
            self.get_parameter("status_timeout_sec").value
        )
        self.action_server_timeout_sec = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self.action_result_timeout_sec = float(
            self.get_parameter("action_result_timeout_sec").value
        )
        self.max_cartesian_state_age_sec = float(
            self.get_parameter("max_cartesian_state_age_sec").value
        )
        self.max_joint_state_age_sec = float(
            self.get_parameter("max_joint_state_age_sec").value
        )
        self.program_result_timeout_sec = float(
            self.get_parameter("program_result_timeout_sec").value
        )
        self.joint_names = tuple(
            str(name).strip()
            for name in self.get_parameter("joint_names").value
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
        self.default_joint_velocity_percent = int(
            self.get_parameter("default_joint_velocity_percent").value
        )
        self.max_joint_velocity_percent = int(
            self.get_parameter("max_joint_velocity_percent").value
        )
        self.max_joint_command_delta_rad = float(
            self.get_parameter("max_joint_command_delta_rad").value
        )
        if min(
            self.status_timeout_sec,
            self.action_server_timeout_sec,
            self.action_result_timeout_sec,
            self.max_cartesian_state_age_sec,
            self.max_joint_state_age_sec,
            self.program_result_timeout_sec,
        ) <= 0.0:
            raise ValueError(
                "All timeout parameters must be greater than zero"
            )
        if len(self.joint_names) != 6 or len(set(self.joint_names)) != 6:
            raise ValueError("joint_names must contain six unique names")
        if not (
            len(self.joint_lower_limits_rad)
            == len(self.joint_upper_limits_rad)
            == len(self.joint_velocity_limits_rad_s)
            == 6
        ):
            raise ValueError("Joint limits must contain six values")
        if not all(
            math.isfinite(low)
            and math.isfinite(high)
            and low < high
            and math.isfinite(velocity)
            and velocity > 0.0
            for low, high, velocity in zip(
                self.joint_lower_limits_rad,
                self.joint_upper_limits_rad,
                self.joint_velocity_limits_rad_s,
            )
        ):
            raise ValueError("Configured joint limits are invalid")
        if not (
            1
            <= self.default_joint_velocity_percent
            <= self.max_joint_velocity_percent
            <= 100
        ):
            raise ValueError("Invalid joint velocity percentages")
        if (
            not math.isfinite(self.max_joint_command_delta_rad)
            or self.max_joint_command_delta_rad <= 0.0
        ):
            raise ValueError("max_joint_command_delta_rad must be positive")

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        status_topic = str(self.get_parameter("status_topic").value)
        cartesian_state_topic = str(
            self.get_parameter("cartesian_state_topic").value
        )
        joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )
        self._latest_status: Optional[DriverStatus] = None
        self._latest_cartesian_state: Optional[CartesianState] = None
        self._cartesian_state_received_at = 0.0
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_received_at = 0.0
        self._status_subscription = self.create_subscription(
            DriverStatus,
            status_topic,
            self._status_callback,
            status_qos,
        )
        self._cartesian_state_subscription = self.create_subscription(
            CartesianState,
            cartesian_state_topic,
            self._cartesian_state_callback,
            10,
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            10,
        )
        self._action_client = ActionClient(
            self,
            JogCartesian,
            self.action_name,
        )
        self._absolute_action_client = ActionClient(
            self,
            MoveCartesian,
            self.absolute_action_name,
        )
        self._trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
        )
        self._program_action_client = ActionClient(
            self,
            RunProgram,
            self.program_action_name,
        )

    def _status_callback(self, message: DriverStatus) -> None:
        self._latest_status = message

    def _cartesian_state_callback(self, message: CartesianState) -> None:
        self._latest_cartesian_state = message
        self._cartesian_state_received_at = time.monotonic()

    def _joint_state_callback(self, message: JointState) -> None:
        self._latest_joint_state = message
        self._joint_state_received_at = time.monotonic()

    @property
    def latest_status(self) -> Optional[DriverStatus]:
        """Return the most recently received driver status."""
        return self._latest_status

    def wait_for_cartesian_state(
        self,
        timeout_sec: Optional[float] = None,
        require_fresh: bool = True,
        received_after: float = 0.0,
    ) -> Optional[CartesianState]:
        """Spin until Cartesian state is available and, optionally, fresh."""
        timeout = (
            self.status_timeout_sec if timeout_sec is None else timeout_sec
        )
        deadline = time.monotonic() + max(0.0, timeout)
        while rclpy.ok():
            state = self._latest_cartesian_state
            age = time.monotonic() - self._cartesian_state_received_at
            if state is not None and (
                not require_fresh or age <= self.max_cartesian_state_age_sec
            ) and self._cartesian_state_received_at >= received_after:
                return state
            if time.monotonic() >= deadline:
                return state if not require_fresh else None
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def wait_for_joint_state(
        self,
        timeout_sec: Optional[float] = None,
        require_fresh: bool = True,
        received_after: float = 0.0,
    ) -> Optional[JointState]:
        """Spin until an ordered joint-state sample is available and fresh."""
        timeout = (
            self.status_timeout_sec if timeout_sec is None else timeout_sec
        )
        deadline = time.monotonic() + max(0.0, timeout)
        while rclpy.ok():
            state = self._latest_joint_state
            age = time.monotonic() - self._joint_state_received_at
            if state is not None and (
                not require_fresh or age <= self.max_joint_state_age_sec
            ) and self._joint_state_received_at >= received_after:
                return state
            if time.monotonic() >= deadline:
                return state if not require_fresh else None
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def ordered_joint_positions(
        self,
        message: JointState,
    ) -> tuple[float, float, float, float, float, float]:
        """Reorder one JointState into the configured six-joint order."""
        if len(message.name) != len(message.position):
            raise RuntimeError("JointState names and positions have different sizes")
        indexes = {name: index for index, name in enumerate(message.name)}
        missing = [name for name in self.joint_names if name not in indexes]
        if missing:
            raise RuntimeError(
                "JointState is missing configured joints: " + ", ".join(missing)
            )
        positions = tuple(
            float(message.position[indexes[name]]) for name in self.joint_names
        )
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("JointState contains non-finite positions")
        return positions  # type: ignore[return-value]

    def wait_for_driver_status(self) -> DriverStatus:
        """Wait for one driver status sample or raise an actionable error."""
        deadline = time.monotonic() + self.status_timeout_sec
        while (
            rclpy.ok()
            and self._latest_status is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        status = self._latest_status
        if status is None:
            raise RuntimeError(
                "No driver status received; verify that fanucpy bringup is "
                "running"
            )
        return status

    @staticmethod
    def require_connected_motion(status: DriverStatus) -> None:
        """Enforce the common connected and motion-enabled driver gates."""
        if status.state != DriverStatus.CONNECTED:
            raise RuntimeError(
                f"FANUC driver is not connected: {status.message}"
            )
        if not status.motion_commands_enabled:
            raise RuntimeError(
                "Driver motion commands are disabled; restart bringup with "
                "enable_motion_commands:=true"
            )

    def check_execution_ready(self, plan: CartesianJogPlan) -> None:
        """Wait for the driver and validate a plan against its live limits."""
        status = self.wait_for_driver_status()
        self.require_connected_motion(status)

        max_translation = float(status.max_translation_step_mm)
        max_rotation = float(status.max_rotation_step_deg)
        max_velocity = int(status.max_cartesian_velocity_mm_s)
        if max_translation <= 0.0:
            max_translation = self.parser.max_translation_step_mm
        if max_rotation <= 0.0:
            max_rotation = self.parser.max_rotation_step_deg
        if max_velocity <= 0:
            max_velocity = self.parser.max_cartesian_velocity_mm_s

        validate_jog_plan(
            plan,
            max_translation,
            max_rotation,
            max_velocity,
        )

        if not self._action_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            raise RuntimeError(
                "Cartesian jog action server "
                f"'{self.action_name}' is unavailable"
            )

    def check_absolute_cartesian_execution_ready(
        self,
        plan: CartesianTargetPlan,
    ) -> None:
        """Validate a direct absolute target against live driver gates."""
        if not plan.is_complete:
            raise PromptParseError(
                "Partial Cartesian target must be resolved from fresh feedback"
            )
        status = self.wait_for_driver_status()
        self.require_connected_motion(status)
        if not status.absolute_cartesian_commands_enabled:
            raise RuntimeError(
                "Direct absolute Cartesian commands are disabled; restart "
                "bringup with enable_absolute_cartesian_commands:=true"
            )
        max_velocity = int(status.max_cartesian_velocity_mm_s)
        if max_velocity <= 0:
            max_velocity = self.parser.max_cartesian_velocity_mm_s
        validate_cartesian_target_plan(plan, max_velocity)
        if status.absolute_cartesian_bounds_enabled:
            validate_cartesian_target_bounds(
                plan,
                status.absolute_cartesian_lower_bounds,
                status.absolute_cartesian_upper_bounds,
            )
        if not self._absolute_action_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            raise RuntimeError(
                "Absolute Cartesian action server "
                f"'{self.absolute_action_name}' is unavailable"
            )

    def execute(self, plan: CartesianJogPlan) -> bool:
        """Send one previously checked plan and wait for its action result."""
        goal = JogCartesian.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        (
            goal.delta_x_mm,
            goal.delta_y_mm,
            goal.delta_z_mm,
            goal.delta_w_deg,
            goal.delta_p_deg,
            goal.delta_r_deg,
        ) = plan.offset
        goal.velocity_mm_s = plan.velocity_mm_s

        self.get_logger().warning(
            "Sending one bounded prompt motion. Terminal interruption does "
            "not guarantee that controller motion will stop."
        )
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self.action_server_timeout_sec,
        )
        if not send_future.done():
            self.get_logger().error(
                "Timed out while sending the jog action goal"
            )
            return False

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send prompt motion: {exc}")
            return False
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Prompt motion was rejected; inspect the driver log and limits"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.action_result_timeout_sec,
        )
        if not result_future.done():
            self.get_logger().error(
                "Timed out waiting for the jog result; the controller may "
                "still be moving. Use pendant HOLD or emergency stop if "
                "motion must stop."
            )
            return False

        try:
            result = result_future.result().result
        except Exception as exc:
            self.get_logger().error(f"Prompt motion result failed: {exc}")
            return False

        if not result.success:
            self.get_logger().error(result.message)
            return False

        self.get_logger().info(
            "Prompt motion completed at [X Y Z W P R] = "
            f"[{result.target_x_mm:.3f}, {result.target_y_mm:.3f}, "
            f"{result.target_z_mm:.3f}, {result.target_w_deg:.3f}, "
            f"{result.target_p_deg:.3f}, {result.target_r_deg:.3f}]"
        )
        return True

    def execute_absolute_cartesian(
        self,
        plan: CartesianTargetPlan,
    ) -> bool:
        """Send one previously checked direct absolute Cartesian target."""
        if not plan.is_complete:
            self.get_logger().error(
                "Refusing unresolved partial absolute Cartesian target"
            )
            return False
        goal = MoveCartesian.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        (
            goal.target_x_mm,
            goal.target_y_mm,
            goal.target_z_mm,
            goal.target_w_deg,
            goal.target_p_deg,
            goal.target_r_deg,
        ) = plan.target
        goal.velocity_mm_s = plan.velocity_mm_s

        self.get_logger().warning(
            "Sending one direct absolute Cartesian target. Relative jog-step "
            "limits do not apply, and terminal interruption does not stop motion."
        )
        send_future = self._absolute_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self.action_server_timeout_sec,
        )
        if not send_future.done():
            self.get_logger().error(
                "Timed out while sending the absolute Cartesian goal"
            )
            return False
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to send absolute Cartesian target: {exc}"
            )
            return False
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Absolute Cartesian target was rejected; inspect driver logs"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.action_result_timeout_sec,
        )
        if not result_future.done():
            self.get_logger().error(
                "Timed out waiting for the absolute Cartesian result; the "
                "controller may still be moving. Use pendant HOLD or emergency "
                "stop if motion must stop."
            )
            return False
        try:
            result = result_future.result().result
        except Exception as exc:
            self.get_logger().error(
                f"Absolute Cartesian result failed: {exc}"
            )
            return False
        if not result.success:
            self.get_logger().error(result.message)
            return False
        self.get_logger().info(
            "Absolute Cartesian target completed at [X Y Z W P R] = "
            f"[{result.target_x_mm:.3f}, {result.target_y_mm:.3f}, "
            f"{result.target_z_mm:.3f}, {result.target_w_deg:.3f}, "
            f"{result.target_p_deg:.3f}, {result.target_r_deg:.3f}]"
        )
        return True

    def check_joint_execution_ready(
        self,
        plan: JointTargetPlan,
        current_positions_rad: Sequence[float],
    ) -> tuple[
        tuple[float, float, float, float, float, float],
        int,
    ]:
        """Validate a direct joint target against feedback and live gates."""
        status = self.wait_for_driver_status()
        self.require_connected_motion(status)
        if (
            status.trajectory_action_name.strip()
            and status.trajectory_action_name != self.trajectory_action_name
        ):
            raise RuntimeError(
                "Configured trajectory action does not match the driver: "
                f"{self.trajectory_action_name!r} != "
                f"{status.trajectory_action_name!r}"
            )

        maximum_velocity = int(status.max_joint_velocity_percent)
        if maximum_velocity <= 0:
            maximum_velocity = self.max_joint_velocity_percent
        delta_limit = self.max_joint_command_delta_rad
        if (
            status.goal_only_execution_enabled
            and status.goal_only_max_joint_delta_rad > 0.0
        ):
            delta_limit = min(
                delta_limit,
                float(status.goal_only_max_joint_delta_rad),
            )
        target = validate_joint_target_plan(
            plan,
            self.joint_lower_limits_rad,
            self.joint_upper_limits_rad,
            maximum_velocity,
            current_positions_rad=current_positions_rad,
            max_delta_rad=delta_limit,
        )
        resolved_velocity = int(plan.velocity_percent)
        if resolved_velocity == 0:
            resolved_velocity = int(status.default_joint_velocity_percent)
            if resolved_velocity <= 0:
                resolved_velocity = self.default_joint_velocity_percent
        if not 1 <= resolved_velocity <= maximum_velocity:
            raise PromptParseError(
                "Resolved joint velocity must be in the range "
                f"1..{maximum_velocity} percent"
            )
        if not self._trajectory_action_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            raise RuntimeError(
                "FollowJointTrajectory action server "
                f"'{self.trajectory_action_name}' is unavailable"
            )
        return target, resolved_velocity

    @staticmethod
    def _set_duration(message: Any, seconds: float) -> None:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        message.sec = whole_seconds
        message.nanosec = nanoseconds

    def execute_joint_target(
        self,
        plan: JointTargetPlan,
        current_positions_rad: Sequence[float],
        target_positions_rad: Sequence[float],
        velocity_percent: int,
    ) -> bool:
        """Send one two-point absolute joint target trajectory."""
        duration_sec = joint_motion_duration_sec(
            current_positions_rad,
            target_positions_rad,
            self.joint_velocity_limits_rad_s,
            velocity_percent,
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        start = JointTrajectoryPoint()
        start.positions = list(current_positions_rad)
        self._set_duration(start.time_from_start, 0.0)
        target = JointTrajectoryPoint()
        target.positions = list(target_positions_rad)
        self._set_duration(target.time_from_start, duration_sec)
        goal.trajectory.points = [start, target]

        self.get_logger().warning(
            "Sending one bounded absolute joint target. Terminal interruption "
            "does not guarantee that controller motion will stop."
        )
        send_future = self._trajectory_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self.action_server_timeout_sec,
        )
        if not send_future.done():
            self.get_logger().error("Timed out while sending the joint goal")
            return False
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send joint target: {exc}")
            return False
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "Joint target was rejected; inspect driver logs and limits"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.action_result_timeout_sec,
        )
        if not result_future.done():
            self.get_logger().error(
                "Timed out waiting for the joint result; the controller may "
                "still be moving. Use pendant HOLD or emergency stop if needed."
            )
            return False
        try:
            result = result_future.result().result
        except Exception as exc:
            self.get_logger().error(f"Joint target result failed: {exc}")
            return False
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                result.error_string or f"Joint result code {result.error_code}"
            )
            return False
        self.get_logger().info(
            "Absolute joint target completed at "
            f"{velocity_percent}%: {result.error_string}"
        )
        return True

    def check_program_execution_ready(self, plan: TpProgramPlan) -> TpProgramPlan:
        """Validate a TP program against the live driver allowlist and gate."""
        validated = validate_tp_program_plan(plan)
        status = self.wait_for_driver_status()
        self.require_connected_motion(status)
        if not status.program_execution_enabled:
            raise RuntimeError(
                "TP program execution is disabled; restart bringup with "
                "enable_program_execution:=true and an explicit allowlist"
            )
        allowed = tuple(str(name).strip().upper() for name in status.allowed_tp_programs)
        if validated.program_name not in allowed:
            raise RuntimeError(
                f"TP program {validated.program_name} is not in the live "
                f"driver allowlist: {list(allowed)}"
            )
        if not self._program_action_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            raise RuntimeError(
                f"TP program action '{self.program_action_name}' is unavailable"
            )
        return validated

    def execute_tp_program(self, plan: TpProgramPlan) -> bool:
        """Send one previously checked TP program call and never repeat it."""
        goal = RunProgram.Goal()
        goal.program_name = plan.program_name
        self.get_logger().warning(
            f"Sending one allowlisted TP program call: {plan.program_name}. "
            "This client will not retry the program automatically."
        )
        send_future = self._program_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=self.action_server_timeout_sec,
        )
        if not send_future.done():
            self.get_logger().error("Timed out while sending the TP program goal")
            return False
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send TP program: {exc}")
            return False
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "TP program was rejected; inspect the gate and allowlist"
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=self.program_result_timeout_sec,
        )
        if not result_future.done():
            self.get_logger().error(
                "Timed out waiting for the TP program result. Do not repeat "
                "the program automatically; inspect the robot first."
            )
            return False
        try:
            result = result_future.result().result
        except Exception as exc:
            self.get_logger().error(f"TP program result failed: {exc}")
            return False
        if not result.success:
            self.get_logger().error(
                f"TP program {plan.program_name} failed "
                f"({result.response_code}): {result.message}"
            )
            return False
        recovery_failed = any(
            phrase in result.message.lower()
            for phrase in ("recovery failed", "connection refused", "not ready")
        )
        if recovery_failed:
            self.get_logger().error(
                f"TP program reported success but session recovery is not "
                f"ready: {result.message}. Do not repeat the program."
            )
            return False
        self.get_logger().info(
            f"TP program {plan.program_name} completed: {result.message}"
        )
        return True

    def _feedback_callback(self, feedback_message: Any) -> None:
        self.get_logger().debug(feedback_message.feedback.state)

    def destroy_node(self) -> None:
        """Destroy the action client before normal node entities."""
        self._program_action_client.destroy()
        self._trajectory_action_client.destroy()
        self._absolute_action_client.destroy()
        self._action_client.destroy()
        super().destroy_node()


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Preview or execute one deterministic Cartesian prompt through "
            "/fanuc/jog_cartesian. Execution always requires typed "
            "confirmation."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "request real execution after live driver checks and confirmation"
        ),
    )
    parser.add_argument(
        "prompt",
        nargs="+",
        help='for example: "move left 10 mm and up 5 mm at 20 mm/s"',
    )
    return parser


def _cli_arguments(args: Optional[Sequence[str]]) -> Sequence[str]:
    raw_args = list(sys.argv) if args is None else [sys.argv[0], *args]
    return remove_ros_args(args=raw_args)[1:]


def run(args: Optional[Sequence[str]] = None) -> int:
    """Run the prompt CLI and return a process-style exit code."""
    options = _argument_parser().parse_args(_cli_arguments(args))
    prompt = " ".join(options.prompt)

    rclpy.init(args=args)
    node: Optional[FanucpyPromptControl] = None
    try:
        node = FanucpyPromptControl()
        try:
            plan = node.parser.parse(prompt)
        except PromptParseError as exc:
            node.get_logger().error(f"Prompt rejected: {exc}")
            return 2

        print("\nParsed Cartesian plan")
        print("---------------------")
        print(format_plan(plan, node.frame_id))

        if not options.execute:
            print(
                "\nDRY RUN: nothing was sent to the robot. Add --execute only "
                "after reviewing the frame and offset."
            )
            return 0

        try:
            node.check_execution_ready(plan)
        except (PromptParseError, RuntimeError) as exc:
            node.get_logger().error(f"Execution blocked: {exc}")
            return 2

        if not sys.stdin.isatty():
            node.get_logger().error(
                "Execution requires an interactive terminal for confirmation"
            )
            return 2

        print(
            "\nReview the active FANUC frame, safeguarded workspace, tool, "
            "and controller speed."
        )
        confirmation = input("Type EXECUTE to send this one motion goal: ")
        if confirmation != "EXECUTE":
            print("Motion cancelled; nothing was sent.")
            return 1

        return 0 if node.execute(plan) else 1
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Interrupted locally. This does not guarantee active robot "
                "motion has stopped; use controller HOLD or emergency stop "
                "when required."
            )
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: Optional[Sequence[str]] = None) -> None:
    """Console-script entry point."""
    exit_code = run(args)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
