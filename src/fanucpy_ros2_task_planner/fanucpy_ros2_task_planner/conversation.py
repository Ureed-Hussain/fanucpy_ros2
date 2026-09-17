# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Pure helpers for conversational robot-state and target reporting."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .prompt_parser import CartesianJogPlan
from .task_plans import CartesianTargetPlan, JointTargetPlan


CartesianValues = Tuple[float, float, float, float, float, float]
JointValues = Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class CartesianSnapshot:
    """One controller Cartesian state expressed in millimetres and degrees."""

    frame_id: str
    x_mm: float
    y_mm: float
    z_mm: float
    w_deg: float
    p_deg: float
    r_deg: float

    @property
    def values(self) -> CartesianValues:
        """Return state in FANUC X/Y/Z/W/P/R order."""
        return (
            self.x_mm,
            self.y_mm,
            self.z_mm,
            self.w_deg,
            self.p_deg,
            self.r_deg,
        )

    def target_for(self, plan: CartesianJogPlan) -> CartesianValues:
        """Add a relative jog to the snapshot for operator preview."""
        return tuple(
            current + delta
            for current, delta in zip(self.values, plan.offset)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class JointSnapshot:
    """One ordered ROS joint-state sample stored internally in radians."""

    names: Tuple[str, str, str, str, str, str]
    positions_rad: JointValues

    @property
    def positions_deg(self) -> JointValues:
        """Return current positions in operator-friendly degrees."""
        return tuple(
            math.degrees(value) for value in self.positions_rad
        )  # type: ignore[return-value]


def format_cartesian_values(values: CartesianValues) -> str:
    """Format X/Y/Z/W/P/R with stable precision and units."""
    formatted = ", ".join(f"{value:.3f}" for value in values)
    return f"[{formatted}] [mm, deg]"


def format_snapshot(snapshot: CartesianSnapshot) -> str:
    """Format one current controller state."""
    return (
        f"Current [{snapshot.frame_id}] [X Y Z W P R]: "
        f"{format_cartesian_values(snapshot.values)}"
    )


def format_target(
    snapshot: Optional[CartesianSnapshot],
    plan: CartesianJogPlan,
) -> str:
    """Format the calculated target or report unavailable feedback."""
    if snapshot is None:
        return (
            "Calculated target: unavailable because no Cartesian state was "
            "received"
        )
    return (
        f"Calculated target [{snapshot.frame_id}] [X Y Z W P R]: "
        f"{format_cartesian_values(snapshot.target_for(plan))}"
    )


def format_absolute_cartesian_target(
    frame_id: str,
    plan: CartesianTargetPlan,
) -> str:
    """Format a model-proposed absolute Cartesian target."""
    return (
        f"Absolute target [{frame_id}] [X Y Z W P R]: "
        f"{format_cartesian_values(plan.target)}"
    )


def format_partial_cartesian_target(
    frame_id: str,
    plan: CartesianTargetPlan,
) -> str:
    """Format axes explicitly supplied in a partial absolute request."""
    labels = ("X", "Y", "Z", "W", "P", "R")
    units = ("mm", "mm", "mm", "deg", "deg", "deg")
    requested = [
        f"{label}={value:.3f} {unit}"
        for label, unit, value, is_set in zip(
            labels,
            units,
            plan.target,
            plan.specified_mask,
        )
        if is_set
    ]
    preserved = [
        label
        for label, is_set in zip(labels, plan.specified_mask)
        if not is_set
    ]
    text = f"Requested absolute axes [{frame_id}]: " + ", ".join(requested)
    if preserved:
        text += "; preserve current " + "/".join(preserved)
    return text


def format_joint_values(values_deg: Sequence[float]) -> str:
    """Format six joint values in degrees."""
    return "[" + ", ".join(f"{value:.3f}" for value in values_deg) + "] deg"


def format_joint_snapshot(snapshot: JointSnapshot) -> str:
    """Format current joint names and positions."""
    return (
        f"Current joints [{' '.join(snapshot.names)}]: "
        f"{format_joint_values(snapshot.positions_deg)}"
    )


def format_joint_target(names: Sequence[str], plan: JointTargetPlan) -> str:
    """Format an absolute conversational joint target."""
    return (
        f"Absolute joint target [{' '.join(names)}]: "
        f"{format_joint_values(plan.positions_deg)}"
    )


def model_robot_context(
    snapshot: Optional[CartesianSnapshot],
    driver_state: str,
    motion_enabled: bool,
    frame_id: str,
    max_translation_step_mm: float,
    max_rotation_step_deg: float,
    max_velocity_mm_s: int,
    joint_snapshot: Optional[JointSnapshot] = None,
    max_joint_delta_rad: float = 0.35,
    default_joint_velocity_percent: int = 5,
    max_joint_velocity_percent: int = 10,
    absolute_cartesian_enabled: bool = False,
    absolute_cartesian_bounds_enabled: bool = False,
    program_execution_enabled: bool = False,
    allowed_tp_programs: Sequence[str] = (),
) -> str:
    """Create trusted runtime context supplied separately from user text."""
    if snapshot is None:
        position = "Cartesian state is unavailable."
    else:
        position = format_snapshot(snapshot)
    if joint_snapshot is None:
        joint_position = "Joint state is unavailable."
    else:
        joint_position = format_joint_snapshot(joint_snapshot)
    allowed_programs = (
        ", ".join(str(name) for name in allowed_tp_programs)
        if allowed_tp_programs
        else "none"
    )
    return (
        f"Driver state: {driver_state}\n"
        f"Motion gate enabled: {motion_enabled}\n"
        f"Command frame: {frame_id}\n"
        f"{position}\n"
        f"Per-axis translation limit: {max_translation_step_mm:g} mm\n"
        f"Per-axis rotation limit: {max_rotation_step_deg:g} degrees\n"
        f"Cartesian velocity limit: {max_velocity_mm_s} mm/s\n"
        "Direct absolute Cartesian gate enabled: "
        f"{absolute_cartesian_enabled}\n"
        "Absolute Cartesian coordinate bounds enabled: "
        f"{absolute_cartesian_bounds_enabled}\n"
        f"{joint_position}\n"
        f"Per-joint command delta limit: {max_joint_delta_rad:g} rad\n"
        f"Default joint velocity: {default_joint_velocity_percent}%\n"
        f"Maximum joint velocity: {max_joint_velocity_percent}%\n"
        f"TP program gate enabled: {program_execution_enabled}\n"
        f"Allowed TP programs: {allowed_programs}"
    )
