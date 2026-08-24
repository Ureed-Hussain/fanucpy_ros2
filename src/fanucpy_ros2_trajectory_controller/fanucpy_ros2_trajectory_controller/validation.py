# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Pure validation and speed-selection helpers for joint trajectories."""

from dataclasses import dataclass
import math
from typing import Any, Optional, Sequence, Tuple


JointValues = Tuple[float, ...]


@dataclass(frozen=True)
class ValidatedTrajectoryPoint:
    """One trajectory point reordered into the driver's canonical joint order."""

    positions: JointValues
    velocities: Optional[JointValues]
    accelerations: Optional[JointValues]
    time_from_start_sec: float


@dataclass(frozen=True)
class ValidatedTrajectory:
    """A finite, bounded trajectory ready for sequential controller execution."""

    joint_names: Tuple[str, ...]
    points: Tuple[ValidatedTrajectoryPoint, ...]


def select_execution_points(
    trajectory: ValidatedTrajectory,
    execution_mode: str,
) -> Tuple[Tuple[int, ValidatedTrajectoryPoint], ...]:
    """Select source points commanded by one supported execution mode."""
    if len(trajectory.points) == 0:
        raise ValueError("A validated trajectory must contain at least one point")
    if execution_mode == "stop_at_waypoints":
        return tuple(enumerate(trajectory.points))
    if execution_mode == "goal_only":
        final_index = len(trajectory.points) - 1
        return ((final_index, trajectory.points[final_index]),)
    raise ValueError(
        "Trajectory execution mode must be stop_at_waypoints or goal_only"
    )


def duration_seconds(duration: Any) -> float:
    """Convert a ROS-like Duration message into finite non-negative seconds."""
    seconds = int(duration.sec)
    nanoseconds = int(duration.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("time_from_start must be a valid non-negative duration")
    return seconds + nanoseconds * 1.0e-9


def _finite_reordered_values(
    values: Sequence[float],
    incoming_indexes: Sequence[int],
    field_name: str,
    allow_empty: bool = False,
) -> Optional[JointValues]:
    if allow_empty and len(values) == 0:
        return None
    if len(values) != len(incoming_indexes):
        raise ValueError(
            f"Each trajectory point must contain {len(incoming_indexes)} "
            f"{field_name} values or leave that field empty"
        )
    reordered = tuple(float(values[index]) for index in incoming_indexes)
    if not all(math.isfinite(value) for value in reordered):
        raise ValueError(f"Trajectory {field_name} values must be finite")
    return reordered


def validate_trajectory(
    trajectory: Any,
    expected_joint_names: Sequence[str],
    lower_position_limits_rad: Sequence[float],
    upper_position_limits_rad: Sequence[float],
    velocity_limits_rad_s: Sequence[float],
    max_points: int,
    max_joint_step_rad: float,
) -> ValidatedTrajectory:
    """Validate and reorder a ROS JointTrajectory without importing ROS."""
    expected = tuple(str(name) for name in expected_joint_names)
    incoming = tuple(str(name) for name in trajectory.joint_names)
    joint_count = len(expected)

    if joint_count == 0 or len(set(expected)) != joint_count:
        raise ValueError("Expected joint names must be non-empty and unique")
    if len(incoming) != joint_count or len(set(incoming)) != joint_count:
        raise ValueError("Trajectory joint names must contain each robot joint once")
    if set(incoming) != set(expected):
        raise ValueError("Trajectory joint names do not match the configured robot")
    if not (
        len(lower_position_limits_rad)
        == len(upper_position_limits_rad)
        == len(velocity_limits_rad_s)
        == joint_count
    ):
        raise ValueError("Every configured joint requires position and velocity limits")
    if not 1 <= int(max_points):
        raise ValueError("max_points must be at least one")
    if not math.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0.0:
        raise ValueError("max_joint_step_rad must be finite and greater than zero")
    if not 1 <= len(trajectory.points) <= int(max_points):
        raise ValueError(
            f"Trajectory must contain between 1 and {int(max_points)} points"
        )

    lower = tuple(float(value) for value in lower_position_limits_rad)
    upper = tuple(float(value) for value in upper_position_limits_rad)
    velocity_limits = tuple(float(value) for value in velocity_limits_rad_s)
    if not all(
        math.isfinite(low)
        and math.isfinite(high)
        and low < high
        and math.isfinite(velocity)
        and velocity > 0.0
        for low, high, velocity in zip(lower, upper, velocity_limits)
    ):
        raise ValueError("Configured joint limits are invalid")

    incoming_indexes = tuple(incoming.index(name) for name in expected)
    validated_points = []
    previous_positions: Optional[JointValues] = None
    previous_time = -1.0

    for point_index, point in enumerate(trajectory.points):
        positions = _finite_reordered_values(
            point.positions,
            incoming_indexes,
            "position",
        )
        assert positions is not None
        velocities = _finite_reordered_values(
            point.velocities,
            incoming_indexes,
            "velocity",
            allow_empty=True,
        )
        accelerations = _finite_reordered_values(
            point.accelerations,
            incoming_indexes,
            "acceleration",
            allow_empty=True,
        )
        if len(point.effort) != 0:
            raise ValueError("Effort trajectory commands are not supported")

        point_time = duration_seconds(point.time_from_start)
        if point_index > 0 and point_time <= previous_time:
            raise ValueError("Trajectory time_from_start values must strictly increase")

        for joint_index, position in enumerate(positions):
            if not lower[joint_index] <= position <= upper[joint_index]:
                raise ValueError(
                    f"Trajectory position for {expected[joint_index]} exceeds "
                    "the configured joint limits"
                )
        if velocities is not None:
            for joint_index, velocity in enumerate(velocities):
                if abs(velocity) > velocity_limits[joint_index]:
                    raise ValueError(
                        f"Trajectory velocity for {expected[joint_index]} exceeds "
                        "the configured joint limit"
                    )

        if previous_positions is not None and any(
            abs(current - previous) > max_joint_step_rad
            for current, previous in zip(positions, previous_positions)
        ):
            raise ValueError(
                "Adjacent trajectory points exceed max_joint_step_rad; "
                "increase trajectory sampling rather than skipping the planned path"
            )

        validated_points.append(
            ValidatedTrajectoryPoint(
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                time_from_start_sec=point_time,
            )
        )
        previous_positions = positions
        previous_time = point_time

    return ValidatedTrajectory(expected, tuple(validated_points))


def validate_start_position(
    actual_positions_rad: Sequence[float],
    first_positions_rad: Sequence[float],
    tolerance_rad: float,
) -> JointValues:
    """Return start errors or reject a trajectory that starts away from the robot."""
    if len(actual_positions_rad) != len(first_positions_rad):
        raise ValueError("Current and trajectory joint counts do not match")
    if not math.isfinite(tolerance_rad) or tolerance_rad <= 0.0:
        raise ValueError("Start tolerance must be finite and greater than zero")
    errors = tuple(
        float(target) - float(actual)
        for target, actual in zip(first_positions_rad, actual_positions_rad)
    )
    if not all(math.isfinite(error) for error in errors):
        raise ValueError("Current robot positions must be finite")
    if any(abs(error) > tolerance_rad for error in errors):
        raise ValueError(
            "Trajectory start differs from the current robot state by more than "
            f"{tolerance_rad:.6f} rad"
        )
    return errors


def validate_goal_only_distance(
    actual_positions_rad: Sequence[float],
    goal_positions_rad: Sequence[float],
    maximum_delta_rad: float,
    joint_names: Optional[Sequence[str]] = None,
) -> JointValues:
    """Reject a direct goal whose per-joint distance exceeds the configured cap."""
    if len(actual_positions_rad) != len(goal_positions_rad):
        raise ValueError("Current and goal joint counts do not match")
    if joint_names is not None and len(joint_names) != len(goal_positions_rad):
        raise ValueError("Joint names and goal joint counts do not match")
    if not math.isfinite(maximum_delta_rad) or maximum_delta_rad <= 0.0:
        raise ValueError(
            "Goal-only maximum joint delta must be finite and greater than zero"
        )

    deltas = tuple(
        float(goal) - float(actual)
        for goal, actual in zip(goal_positions_rad, actual_positions_rad)
    )
    if not all(math.isfinite(delta) for delta in deltas):
        raise ValueError("Current and goal joint positions must be finite")

    for index, delta in enumerate(deltas):
        if abs(delta) > maximum_delta_rad:
            joint_name = (
                str(joint_names[index])
                if joint_names is not None
                else f"joint index {index}"
            )
            raise ValueError(
                f"Goal-only direct move for {joint_name} is "
                f"{abs(delta):.6f} rad, exceeding the configured "
                f"{maximum_delta_rad:.6f} rad limit"
            )
    return deltas


def segment_velocity_percent(
    previous_positions_rad: Sequence[float],
    point: ValidatedTrajectoryPoint,
    previous_time_sec: float,
    velocity_limits_rad_s: Sequence[float],
    default_percent: int,
    maximum_percent: int,
) -> int:
    """Approximate a MoveIt segment's speed using the FANUC percent command."""
    if not 1 <= int(default_percent) <= int(maximum_percent) <= 100:
        raise ValueError("Joint velocity percentages must satisfy 1 <= default <= max <= 100")
    if not (
        len(previous_positions_rad)
        == len(point.positions)
        == len(velocity_limits_rad_s)
    ):
        raise ValueError("Joint counts do not match while selecting segment velocity")

    fractions = []
    duration = point.time_from_start_sec - float(previous_time_sec)
    if duration > 0.0:
        fractions.extend(
            abs(target - previous) / duration / float(limit)
            for target, previous, limit in zip(
                point.positions,
                previous_positions_rad,
                velocity_limits_rad_s,
            )
        )
    if point.velocities is not None:
        fractions.extend(
            abs(velocity) / float(limit)
            for velocity, limit in zip(point.velocities, velocity_limits_rad_s)
        )

    requested = int(default_percent)
    if fractions and max(fractions) > 0.0:
        requested = max(1, int(math.ceil(max(fractions) * 100.0)))
    return min(requested, int(maximum_percent))


def position_errors(
    desired_positions_rad: Sequence[float],
    actual_positions_rad: Sequence[float],
) -> JointValues:
    """Return desired-minus-actual joint position errors."""
    if len(desired_positions_rad) != len(actual_positions_rad):
        raise ValueError("Desired and actual joint counts do not match")
    errors = tuple(
        float(desired) - float(actual)
        for desired, actual in zip(desired_positions_rad, actual_positions_rad)
    )
    if not all(math.isfinite(error) for error in errors):
        raise ValueError("Joint position errors must be finite")
    return errors
