# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Standard FollowJointTrajectory action server embedded in the FANUC driver."""

from dataclasses import dataclass
import math
import time
from typing import Any, Callable, Optional, Sequence, Tuple

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from .validation import (
    ValidatedTrajectory,
    ValidatedTrajectoryPoint,
    position_errors,
    select_execution_points,
    segment_velocity_percent,
    validate_goal_only_distance,
    validate_start_position,
    validate_trajectory,
)


CanExecuteCallback = Callable[[], Tuple[bool, str]]
ReserveCallback = Callable[[], bool]
ReleaseCallback = Callable[[], None]
CurrentPositionsCallback = Callable[[], Optional[Tuple[float, ...]]]
ExecuteWaypointCallback = Callable[
    [Sequence[float], int], Tuple[Tuple[float, ...], str]
]


@dataclass(frozen=True)
class TrajectoryControllerConfig:
    """Configuration required by one standard trajectory action server."""

    action_name: str
    joint_names: Tuple[str, ...]
    lower_position_limits_rad: Tuple[float, ...]
    upper_position_limits_rad: Tuple[float, ...]
    velocity_limits_rad_s: Tuple[float, ...]
    max_points: int
    max_joint_step_rad: float
    start_tolerance_rad: float
    path_tolerance_rad: float
    goal_tolerance_rad: float
    default_velocity_percent: int
    maximum_velocity_percent: int
    execution_mode: str
    allow_goal_only_execution: bool
    goal_only_max_joint_delta_rad: float


class FanucpyFollowJointTrajectoryServer:
    """Expose MoveIt's standard action while sharing the driver's connection."""

    def __init__(
        self,
        node: Node,
        config: TrajectoryControllerConfig,
        can_execute: CanExecuteCallback,
        reserve_motion: ReserveCallback,
        release_motion: ReleaseCallback,
        current_positions: CurrentPositionsCallback,
        execute_waypoint: ExecuteWaypointCallback,
    ) -> None:
        self._node = node
        self._config = config
        self._can_execute = can_execute
        self._reserve_motion = reserve_motion
        self._release_motion = release_motion
        self._current_positions = current_positions
        self._execute_waypoint = execute_waypoint
        self._validate_config()
        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            config.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def _validate_config(self) -> None:
        config = self._config
        joint_count = len(config.joint_names)
        if not config.action_name.strip():
            raise ValueError("FollowJointTrajectory action name must not be empty")
        if joint_count == 0 or len(set(config.joint_names)) != joint_count:
            raise ValueError("Trajectory controller joint names must be unique")
        if not (
            len(config.lower_position_limits_rad)
            == len(config.upper_position_limits_rad)
            == len(config.velocity_limits_rad_s)
            == joint_count
        ):
            raise ValueError("Trajectory controller limits must match its joints")
        if not (
            1
            <= config.default_velocity_percent
            <= config.maximum_velocity_percent
            <= 100
        ):
            raise ValueError("Invalid default or maximum joint velocity percentage")
        if config.execution_mode not in ("stop_at_waypoints", "goal_only"):
            raise ValueError(
                "Trajectory execution mode must be stop_at_waypoints or goal_only"
            )
        if (
            not math.isfinite(config.goal_only_max_joint_delta_rad)
            or config.goal_only_max_joint_delta_rad <= 0.0
        ):
            raise ValueError(
                "goal_only_max_joint_delta_rad must be greater than zero"
            )
        for name, value in (
            ("start_tolerance_rad", config.start_tolerance_rad),
            ("path_tolerance_rad", config.path_tolerance_rad),
            ("goal_tolerance_rad", config.goal_tolerance_rad),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")

    def _validated_request(self, request: Any) -> ValidatedTrajectory:
        header_stamp = request.trajectory.header.stamp
        if int(header_stamp.sec) != 0 or int(header_stamp.nanosec) != 0:
            raise ValueError(
                "Only immediate trajectories with a zero header stamp are supported"
            )
        if len(request.path_tolerance) != 0 or len(request.goal_tolerance) != 0:
            raise ValueError(
                "Per-goal tolerances are not supported by the fanucpy bridge; "
                "use the driver-configured path and goal tolerances"
            )
        if (
            len(request.component_path_tolerance) != 0
            or len(request.component_goal_tolerance) != 0
        ):
            raise ValueError("Multi-DOF component tolerances are not supported")
        if (
            len(request.multi_dof_trajectory.joint_names) != 0
            or len(request.multi_dof_trajectory.points) != 0
        ):
            raise ValueError("Multi-DOF trajectories are not supported")
        if (
            int(request.goal_time_tolerance.sec) != 0
            or int(request.goal_time_tolerance.nanosec) != 0
        ):
            raise ValueError(
                "goal_time_tolerance is not supported because MAPPDK does not "
                "provide timed trajectory execution"
            )
        return validate_trajectory(
            request.trajectory,
            self._config.joint_names,
            self._config.lower_position_limits_rad,
            self._config.upper_position_limits_rad,
            self._config.velocity_limits_rad_s,
            self._config.max_points,
            self._config.max_joint_step_rad,
        )

    def _validate_execution_selection(
        self,
        current: Sequence[float],
        trajectory: ValidatedTrajectory,
    ) -> None:
        if self._config.execution_mode != "goal_only":
            return
        if not self._config.allow_goal_only_execution:
            raise ValueError(
                "goal_only execution requires allow_goal_only_execution=true"
            )
        validate_goal_only_distance(
            current,
            trajectory.points[-1].positions,
            self._config.goal_only_max_joint_delta_rad,
            self._config.joint_names,
        )

    def _goal_callback(self, goal_request: Any) -> GoalResponse:
        allowed, reason = self._can_execute()
        if not allowed:
            self._node.get_logger().warning(
                f"Rejected FollowJointTrajectory goal: {reason}"
            )
            return GoalResponse.REJECT

        try:
            trajectory = self._validated_request(goal_request)
            current = self._current_positions()
            if current is None:
                raise ValueError("No current joint-state sample is available")
            validate_start_position(
                current,
                trajectory.points[0].positions,
                self._config.start_tolerance_rad,
            )
            self._validate_execution_selection(current, trajectory)
        except ValueError as exc:
            self._node.get_logger().warning(
                f"Rejected FollowJointTrajectory goal: {exc}"
            )
            return GoalResponse.REJECT

        if not self._reserve_motion():
            self._node.get_logger().warning(
                "Rejected FollowJointTrajectory goal: another motion is executing"
            )
            return GoalResponse.REJECT
        if self._config.execution_mode == "goal_only":
            self._node.get_logger().warning(
                "Accepted GOAL-ONLY trajectory: commanding only the final "
                f"joint target and skipping {len(trajectory.points) - 1} "
                "MoveIt points; the direct FANUC move does not follow the "
                "collision-checked MoveIt path"
            )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle: Any) -> CancelResponse:
        self._node.get_logger().warning(
            "Trajectory cancellation rejected: fanucpy/MAPPDK has no dependable "
            "motion-abort command; use teach-pendant HOLD or the emergency stop"
        )
        return CancelResponse.REJECT

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    def _publish_feedback(
        self,
        goal_handle: Any,
        desired: ValidatedTrajectoryPoint,
        actual_positions: Sequence[float],
        elapsed_sec: float,
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self._node.get_clock().now().to_msg()
        feedback.joint_names = list(self._config.joint_names)

        feedback.desired = JointTrajectoryPoint()
        feedback.desired.positions = list(desired.positions)
        if desired.velocities is not None:
            feedback.desired.velocities = list(desired.velocities)
        if desired.accelerations is not None:
            feedback.desired.accelerations = list(desired.accelerations)
        feedback.desired.time_from_start = self._duration_from_seconds(
            desired.time_from_start_sec
        )

        feedback.actual = JointTrajectoryPoint()
        feedback.actual.positions = list(actual_positions)
        feedback.actual.time_from_start = self._duration_from_seconds(elapsed_sec)

        feedback.error = JointTrajectoryPoint()
        feedback.error.positions = list(
            position_errors(desired.positions, actual_positions)
        )
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _result(code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = int(code)
        result.error_string = str(message)
        return result

    def _execute_callback(self, goal_handle: Any) -> FollowJointTrajectory.Result:
        started = time.monotonic()
        try:
            trajectory = self._validated_request(goal_handle.request)
            actual = self._current_positions()
            if actual is None:
                raise RuntimeError("Robot joint state became unavailable")
            validate_start_position(
                actual,
                trajectory.points[0].positions,
                self._config.start_tolerance_rad,
            )
            self._validate_execution_selection(actual, trajectory)

            previous_positions = tuple(actual)
            previous_time = 0.0
            execution_points = select_execution_points(
                trajectory,
                self._config.execution_mode,
            )

            for command_index, (point_index, point) in enumerate(
                execution_points
            ):
                allowed, reason = self._can_execute()
                if not allowed:
                    raise RuntimeError(reason)

                start_errors = position_errors(point.positions, previous_positions)
                is_last_command = command_index == len(execution_points) - 1
                tolerance = (
                    self._config.goal_tolerance_rad
                    if is_last_command
                    else self._config.path_tolerance_rad
                )
                can_skip_satisfied_target = (
                    self._config.execution_mode == "goal_only"
                    or point_index == 0
                )
                if can_skip_satisfied_target and all(
                    abs(error)
                    <= min(self._config.start_tolerance_rad, tolerance)
                    for error in start_errors
                ):
                    actual = previous_positions
                else:
                    velocity_percent = segment_velocity_percent(
                        previous_positions,
                        point,
                        previous_time,
                        self._config.velocity_limits_rad_s,
                        self._config.default_velocity_percent,
                        self._config.maximum_velocity_percent,
                    )
                    actual, response_message = self._execute_waypoint(
                        point.positions,
                        velocity_percent,
                    )
                    if self._config.execution_mode == "goal_only":
                        self._node.get_logger().info(
                            "Goal-only final target completed at "
                            f"{velocity_percent}%: {response_message}"
                        )
                    else:
                        self._node.get_logger().info(
                            f"Trajectory waypoint {point_index + 1}/"
                            f"{len(trajectory.points)} completed at "
                            f"{velocity_percent}%: {response_message}"
                        )

                errors = position_errors(point.positions, actual)
                if any(abs(error) > tolerance for error in errors):
                    error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        if is_last_command
                        else FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    )
                    target_label = (
                        "Goal-only final target"
                        if self._config.execution_mode == "goal_only"
                        else f"Waypoint {point_index + 1}"
                    )
                    result = self._result(
                        error_code,
                        f"{target_label} exceeded the configured "
                        f"{tolerance:.6f} rad position tolerance",
                    )
                    goal_handle.abort()
                    self._node.get_logger().error(result.error_string)
                    return result

                self._publish_feedback(
                    goal_handle,
                    point,
                    actual,
                    time.monotonic() - started,
                )
                previous_positions = tuple(actual)
                previous_time = point.time_from_start_sec

            result = self._result(
                FollowJointTrajectory.Result.SUCCESSFUL,
                (
                    "Direct final joint target completed in goal_only mode"
                    if self._config.execution_mode == "goal_only"
                    else "Trajectory completed in stop_at_waypoints mode"
                ),
            )
            goal_handle.succeed()
            return result
        except ValueError as exc:
            result = self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                str(exc),
            )
            goal_handle.abort()
            self._node.get_logger().error(
                f"FollowJointTrajectory validation failed: {exc}"
            )
            return result
        except Exception as exc:
            result = self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"Trajectory execution failed: {exc}",
            )
            goal_handle.abort()
            self._node.get_logger().error(result.error_string)
            return result
        finally:
            self._release_motion()

    def destroy(self) -> None:
        """Destroy the embedded action server before its owning ROS node."""
        self._action_server.destroy()
