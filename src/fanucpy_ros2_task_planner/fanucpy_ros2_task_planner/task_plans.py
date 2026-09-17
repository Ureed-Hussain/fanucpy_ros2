# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Pure plan types and validation for conversational FANUC tasks."""

from dataclasses import dataclass
import math
import re
from typing import Sequence, Tuple, Union

from .prompt_parser import (
    CartesianJogPlan,
    PromptParseError,
)


CartesianValues = Tuple[float, float, float, float, float, float]
JointValues = Tuple[float, float, float, float, float, float]
_PROGRAM_NAME = re.compile(r"[A-Z][A-Z0-9_]{0,31}")


@dataclass(frozen=True)
class CartesianTargetPlan:
    """One full or partial absolute Cartesian target."""

    x_mm: float
    y_mm: float
    z_mm: float
    w_deg: float
    p_deg: float
    r_deg: float
    velocity_mm_s: int = 0
    x_set: bool = True
    y_set: bool = True
    z_set: bool = True
    w_set: bool = True
    p_set: bool = True
    r_set: bool = True

    @property
    def target(self) -> CartesianValues:
        """Return the target in FANUC X/Y/Z/W/P/R order."""
        return (
            self.x_mm,
            self.y_mm,
            self.z_mm,
            self.w_deg,
            self.p_deg,
            self.r_deg,
        )

    @property
    def specified_mask(self) -> Tuple[bool, bool, bool, bool, bool, bool]:
        """Return which axes were explicitly requested by the operator."""
        return (
            self.x_set,
            self.y_set,
            self.z_set,
            self.w_set,
            self.p_set,
            self.r_set,
        )

    @property
    def is_complete(self) -> bool:
        """Return whether all six target axes have concrete values."""
        return all(self.specified_mask)


@dataclass(frozen=True)
class JointTargetPlan:
    """One absolute J1..J6 target expressed in operator-friendly degrees."""

    joint_1_deg: float
    joint_2_deg: float
    joint_3_deg: float
    joint_4_deg: float
    joint_5_deg: float
    joint_6_deg: float
    velocity_percent: int = 0

    @property
    def positions_deg(self) -> JointValues:
        """Return the target in configured J1..J6 order."""
        return (
            self.joint_1_deg,
            self.joint_2_deg,
            self.joint_3_deg,
            self.joint_4_deg,
            self.joint_5_deg,
            self.joint_6_deg,
        )

    @property
    def positions_rad(self) -> JointValues:
        """Return the target converted to radians for ROS trajectories."""
        return tuple(
            math.radians(value) for value in self.positions_deg
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class TpProgramPlan:
    """One syntactically valid FANUC teach-pendant program request."""

    program_name: str


TaskPlan = Union[
    CartesianJogPlan,
    CartesianTargetPlan,
    JointTargetPlan,
    TpProgramPlan,
]


def validate_cartesian_target_plan(
    plan: CartesianTargetPlan,
    max_velocity_mm_s: int,
) -> CartesianTargetPlan:
    """Validate a finite full or partial target and requested velocity."""
    if not all(isinstance(value, bool) for value in plan.specified_mask):
        raise PromptParseError("Cartesian target axis selectors must be boolean")
    if not any(plan.specified_mask):
        raise PromptParseError(
            "An absolute Cartesian target must specify at least one axis"
        )
    if not all(math.isfinite(float(value)) for value in plan.target):
        raise PromptParseError("Absolute Cartesian target values must be finite")
    velocity = int(plan.velocity_mm_s)
    if velocity != plan.velocity_mm_s:
        raise PromptParseError("Cartesian velocity must be an integer")
    if velocity != 0 and not 1 <= velocity <= int(max_velocity_mm_s):
        raise PromptParseError(
            "Cartesian velocity must be zero for the driver default or in "
            f"the range 1..{int(max_velocity_mm_s)} mm/s"
        )
    return plan


def resolve_cartesian_target(
    plan: CartesianTargetPlan,
    current_values: Sequence[float],
) -> CartesianTargetPlan:
    """Fill omitted target axes from one fresh Cartesian state sample."""
    if len(current_values) != 6:
        raise PromptParseError("Current Cartesian state must contain six values")
    current = tuple(float(value) for value in current_values)
    if not all(math.isfinite(value) for value in current):
        raise PromptParseError("Current Cartesian state values must be finite")
    if not all(isinstance(value, bool) for value in plan.specified_mask):
        raise PromptParseError("Cartesian target axis selectors must be boolean")
    if not any(plan.specified_mask):
        raise PromptParseError(
            "An absolute Cartesian target must specify at least one axis"
        )
    resolved = tuple(
        requested if is_set else actual
        for requested, actual, is_set in zip(
            plan.target,
            current,
            plan.specified_mask,
        )
    )
    return CartesianTargetPlan(
        *resolved,
        velocity_mm_s=plan.velocity_mm_s,
    )


def validate_cartesian_target_bounds(
    plan: CartesianTargetPlan,
    lower_bounds: Sequence[float],
    upper_bounds: Sequence[float],
) -> CartesianTargetPlan:
    """Validate one absolute target against optional driver-reported bounds."""
    if not plan.is_complete:
        raise PromptParseError(
            "Partial Cartesian target must be resolved from current feedback"
        )
    if len(lower_bounds) != 6 or len(upper_bounds) != 6:
        raise PromptParseError("Absolute Cartesian bounds require six values")
    lower = tuple(float(value) for value in lower_bounds)
    upper = tuple(float(value) for value in upper_bounds)
    if not all(
        math.isfinite(low) and math.isfinite(high) and low < high
        for low, high in zip(lower, upper)
    ):
        raise PromptParseError("Absolute Cartesian bounds are invalid")
    for label, value, low, high in zip(
        ("X", "Y", "Z", "W", "P", "R"),
        plan.target,
        lower,
        upper,
    ):
        if not low <= value <= high:
            raise PromptParseError(
                f"Absolute Cartesian {label} target {value:g} is outside "
                f"the configured range [{low:g}, {high:g}]"
            )
    return plan


def validate_joint_target_plan(
    plan: JointTargetPlan,
    lower_limits_rad: Sequence[float],
    upper_limits_rad: Sequence[float],
    max_velocity_percent: int,
    current_positions_rad: Sequence[float] = (),
    max_delta_rad: float = 0.0,
) -> JointValues:
    """Validate an absolute joint target and optional current-to-goal delta."""
    target = plan.positions_rad
    if not all(math.isfinite(value) for value in target):
        raise PromptParseError("Joint target values must be finite")
    if len(lower_limits_rad) != 6 or len(upper_limits_rad) != 6:
        raise PromptParseError("Exactly six joint limits are required")
    lower = tuple(float(value) for value in lower_limits_rad)
    upper = tuple(float(value) for value in upper_limits_rad)
    if not all(
        math.isfinite(low) and math.isfinite(high) and low < high
        for low, high in zip(lower, upper)
    ):
        raise PromptParseError("Configured joint position limits are invalid")
    for index, (value, low, high) in enumerate(
        zip(target, lower, upper),
        start=1,
    ):
        if not low <= value <= high:
            raise PromptParseError(
                f"J{index} target exceeds its configured position limits"
            )

    velocity = int(plan.velocity_percent)
    if velocity != plan.velocity_percent:
        raise PromptParseError("Joint velocity percentage must be an integer")
    if velocity != 0 and not 1 <= velocity <= int(max_velocity_percent):
        raise PromptParseError(
            "Joint velocity must be zero for the driver default or in the "
            f"range 1..{int(max_velocity_percent)} percent"
        )

    if current_positions_rad:
        if len(current_positions_rad) != 6:
            raise PromptParseError("Current joint state must contain six values")
        current = tuple(float(value) for value in current_positions_rad)
        if not all(math.isfinite(value) for value in current):
            raise PromptParseError("Current joint state values must be finite")
        deltas = tuple(
            goal - actual for goal, actual in zip(target, current)
        )
        if all(delta == 0.0 for delta in deltas):
            raise PromptParseError("Joint target is already satisfied")
        if not math.isfinite(max_delta_rad) or max_delta_rad <= 0.0:
            raise PromptParseError(
                "Maximum joint command delta must be greater than zero"
            )
        for index, delta in enumerate(deltas, start=1):
            if abs(delta) > max_delta_rad:
                raise PromptParseError(
                    f"J{index} command delta is {abs(delta):.6f} rad, "
                    f"exceeding the configured {max_delta_rad:.6f} rad limit"
                )
    return target


def joint_motion_duration_sec(
    current_positions_rad: Sequence[float],
    target_positions_rad: Sequence[float],
    velocity_limits_rad_s: Sequence[float],
    velocity_percent: int,
) -> float:
    """Choose trajectory timing that requests a bounded FANUC percentage."""
    if not (
        len(current_positions_rad)
        == len(target_positions_rad)
        == len(velocity_limits_rad_s)
        == 6
    ):
        raise PromptParseError("Joint motion timing requires six joint values")
    if not 1 <= int(velocity_percent) <= 100:
        raise PromptParseError("Resolved joint velocity must be in 1..100 percent")
    fraction = int(velocity_percent) / 100.0
    durations = []
    for actual, target, limit in zip(
        current_positions_rad,
        target_positions_rad,
        velocity_limits_rad_s,
    ):
        actual_value = float(actual)
        target_value = float(target)
        limit_value = float(limit)
        if not all(
            math.isfinite(value)
            for value in (actual_value, target_value, limit_value)
        ) or limit_value <= 0.0:
            raise PromptParseError("Configured joint velocity limits are invalid")
        durations.append(
            abs(target_value - actual_value) / (limit_value * fraction)
        )
    duration = max(durations)
    if duration <= 0.0:
        raise PromptParseError("Joint target is already satisfied")
    # A tiny margin prevents floating-point roundoff in the driver's
    # ceil-based percentage calculation from selecting one percent too high.
    return max(duration * (1.0 + 1.0e-9), 0.001)


def normalize_tp_program_name(program_name: str) -> str:
    """Apply the same public FANUC TP-name constraints as the driver."""
    normalized = str(program_name).strip().upper()
    if not _PROGRAM_NAME.fullmatch(normalized):
        raise PromptParseError(
            "FANUC program name must contain 1..32 letters, digits, or "
            "underscores and must begin with a letter"
        )
    return normalized


def validate_tp_program_plan(plan: TpProgramPlan) -> TpProgramPlan:
    """Return a normalized TP program plan after syntax validation."""
    return TpProgramPlan(normalize_tp_program_name(plan.program_name))
