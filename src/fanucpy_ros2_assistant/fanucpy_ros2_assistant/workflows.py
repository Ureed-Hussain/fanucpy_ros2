# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Deterministic intent and target planning for guarded task workflows."""

from dataclasses import dataclass
import math
import re
from typing import Optional, Sequence

from fanucpy_ros2_task_planner.prompt_parser import PromptParseError
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    validate_cartesian_target_plan,
)

from .vision_motion import (
    VisionMotionError,
    VisionMotionSelection,
    build_vision_cartesian_target,
    has_velocity_request,
    without_velocity_request,
)


PICK_INTENT = "pick"
DROP_TARGET_INTENT = "drop_target"
HOME_PROGRAM_INTENT = "home_program"

_PICK_VERB = re.compile(
    r"\b(?:collect|grab|grasp|pick(?:\s+up)?|retrieve)\b",
    re.IGNORECASE,
)
_DROP_TARGET = re.compile(
    r"\b(?:bring|go|head|move|send|take|travel)\b"
    r".{0,45}\b(?:drop(?:[- ]?off)?)\b"
    r"(?:\s+(?:area|location|place|point|position|station|zone))?\b",
    re.IGNORECASE,
)
_EXPLICIT_DROP_TARGET = re.compile(
    r"\b(?:drop(?:[- ]?off)?\s+"
    r"(?:area|location|place|point|position|station|zone))\b",
    re.IGNORECASE,
)
_DROP_COMMAND = re.compile(
    r"\bdrop(?:\s+off)?\s+(?:it|this|that|the\s+object)\b|"
    r"^\s*(?:please\s+)?drop(?:\s+off)?"
    r"(?=\s*(?:[.!?]|$|please\b|now\b|at\b|with\b))",
    re.IGNORECASE,
)
_HOME_TARGET = re.compile(
    r"\b(?:bring|go|head|move|return|send|take|travel)\b"
    r".{0,35}\bhome(?:\s+position)?\b",
    re.IGNORECASE,
)
_SHORT_HOME = re.compile(
    r"^\s*(?:please\s+)?(?:back\s+)?home\b|"
    r"\bhome\s+(?:position|program)\b",
    re.IGNORECASE,
)
_HOME_PROGRAM = re.compile(
    r"\b(?:call|execute|run|start)\b.{0,20}\bHOME_P\b",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class PickMotionPlan:
    """Three Cartesian stages for an object-centered down/up motion."""

    selection: VisionMotionSelection
    approach: CartesianTargetPlan
    descend: CartesianTargetPlan
    retract: CartesianTargetPlan
    xy_travel_mm: float


def special_intent(prompt: str) -> Optional[str]:
    """Recognize safety-critical named workflows before Ollama is queried."""
    text = " ".join(prompt.strip().split())
    if (
        _HOME_TARGET.search(text)
        or _HOME_PROGRAM.search(text)
        or _SHORT_HOME.search(text)
    ):
        return HOME_PROGRAM_INTENT
    if (
        _DROP_TARGET.search(text)
        or _EXPLICIT_DROP_TARGET.search(text)
        or _DROP_COMMAND.search(text)
    ):
        return DROP_TARGET_INTENT
    if _PICK_VERB.search(text):
        return PICK_INTENT
    return None


def is_workflow_question(prompt: str) -> bool:
    """Identify informational wording without blocking polite commands."""
    return bool(re.search(
        r"\b(?:what|where|which|why|how|explain|describe|tell\s+me|"
        r"show\s+me)\b",
        prompt,
        re.IGNORECASE,
    ))


def workflow_request_issue(prompt: str, intent: str) -> Optional[str]:
    """Reject negated, compound, or unsupported named-target instructions."""
    text = " ".join(prompt.lower().replace("\u2019", "'").split())
    if re.search(
        r"\b(?:don't|dont|do\s+not|not|never|cancel|stop|hold|abort)\b",
        text,
    ):
        return (
            "I have not sent a new command. A negated or stop request does "
            "not authorize this workflow. Use pendant HOLD to stop any "
            "motion already in progress."
        )
    if re.search(
        r"\b(?:then|afterwards?|before|after|if|when|unless|until)\b|"
        r"(?:\band\b|;)\s*(?:also\s+)?"
        r"(?:bring|call|drop|go|grab|head|move|open|pick|release|return|run|"
        r"take)\b",
        text,
    ):
        return (
            "Please request one workflow at a time, for example 'pick "
            "battery one', then 'drop it' after that sequence has completed."
        )
    # Prevent one named workflow from silently taking priority over another.
    families = (
        bool(_PICK_VERB.search(text)),
        bool(_DROP_TARGET.search(text) or _EXPLICIT_DROP_TARGET.search(text)
             or _DROP_COMMAND.search(text)),
        bool(_HOME_TARGET.search(text) or _HOME_PROGRAM.search(text)
             or _SHORT_HOME.search(text)),
    )
    if sum(families) > 1:
        return "Please request either pick, drop, or home in one message."
    if is_workflow_question(text):
        return None
    if intent == PICK_INTENT:
        return None
    if intent == HOME_PROGRAM_INTENT and has_velocity_request(text):
        return (
            "Home calls the configured TP program. Its speed is controlled "
            "by that program and the controller override, so I cannot apply "
            "a Cartesian speed here. Say 'go home' to preview the TP call."
        )
    # A fixed destination must not absorb a request for a different location,
    # explicit coordinates, gripper release, or a different named TP program.
    remainder = without_velocity_request(text)
    if has_velocity_request(remainder):
        return (
            "Please give one speed in mm/s, such as 'drop it at 200 mm/s', "
            "or say 'drop it at max speed'. I will not guess a speed."
        )
    words = set(re.findall(r"[a-z0-9_]+", remainder))
    common = {
        "a", "at", "back", "bring", "can", "could", "for", "go", "head",
        "i", "it", "let", "lets", "me", "move", "my", "now", "our",
        "please", "robot", "s", "send", "take", "that", "the", "this",
        "to", "travel", "us", "will", "with", "would", "you",
    }
    allowed = (
        {"area", "configured", "drop", "location", "object", "off",
         "place", "point", "position", "saved", "station", "zone"}
        if intent == DROP_TARGET_INTENT
        else {"call", "execute", "home", "home_p", "position", "program",
              "return", "run", "start", "tp"}
    )
    if words - common - allowed:
        return (
            "I can use the configured drop position or home program, but "
            "this request adds a destination or instruction I cannot apply. "
            "Try 'drop it at 200 mm/s' or 'go home'. For a different pose, "
            "give a separate absolute Cartesian command."
        )
    return None


def pick_selection_prompt(prompt: str) -> str:
    """Rewrite rough pick wording into the existing object selector grammar."""
    rewritten, count = _PICK_VERB.subn("go to", prompt, count=1)
    if count != 1:
        raise VisionMotionError("The request does not contain a pick intent")
    return rewritten


def _complete_target(
    values: Sequence[float],
    velocity_mm_s: int,
    max_velocity_mm_s: int,
) -> CartesianTargetPlan:
    """Construct and validate one complete Cartesian workflow target."""
    if len(values) != 6:
        raise VisionMotionError("A workflow target requires six values")
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise VisionMotionError("Workflow target values must be finite")
    plan = CartesianTargetPlan(
        x_mm=converted[0],
        y_mm=converted[1],
        z_mm=converted[2],
        w_deg=converted[3],
        p_deg=converted[4],
        r_deg=converted[5],
        velocity_mm_s=int(velocity_mm_s),
    )
    try:
        return validate_cartesian_target_plan(plan, max_velocity_mm_s)
    except PromptParseError as exc:
        raise VisionMotionError(str(exc)) from exc


def build_pick_motion_plan(
    selection: VisionMotionSelection,
    current_values: Sequence[float],
    xy_offset_mm: Sequence[float],
    approach_z_mm: float,
    descend_z_mm: float,
    retract_z_mm: float,
    max_xy_travel_mm: float,
    max_approach_z_change_mm: float,
    max_vertical_stage_mm: float,
    max_velocity_mm_s: int,
) -> PickMotionPlan:
    """Build approach, vertical descent, and vertical retraction targets."""
    if len(current_values) != 6:
        raise VisionMotionError("Current Cartesian state requires six values")
    current = tuple(float(value) for value in current_values)
    if not all(math.isfinite(value) for value in current):
        raise VisionMotionError("Current Cartesian state must be finite")
    z_values = (
        float(approach_z_mm),
        float(descend_z_mm),
        float(retract_z_mm),
    )
    if not all(math.isfinite(value) for value in z_values):
        raise VisionMotionError("Pick Z targets must be finite")
    if z_values[1] >= z_values[0]:
        raise VisionMotionError(
            "Configured pick descent Z must be below the approach Z"
        )
    if z_values[2] < z_values[1]:
        raise VisionMotionError(
            "Configured pick retraction Z must be above the descent Z"
        )
    if not math.isfinite(max_approach_z_change_mm) or (
        max_approach_z_change_mm <= 0.0
    ):
        raise VisionMotionError("Maximum approach Z change must be positive")
    if not math.isfinite(max_vertical_stage_mm) or (
        max_vertical_stage_mm <= 0.0
    ):
        raise VisionMotionError("Maximum vertical stage must be positive")
    approach_z_change = abs(z_values[0] - current[2])
    descent_distance = abs(z_values[1] - z_values[0])
    retraction_distance = abs(z_values[2] - z_values[1])
    if approach_z_change > max_approach_z_change_mm:
        raise VisionMotionError(
            f"Approach Z change {approach_z_change:.3f} mm exceeds the "
            f"configured {max_approach_z_change_mm:.3f} mm limit"
        )
    if max(descent_distance, retraction_distance) > max_vertical_stage_mm:
        raise VisionMotionError(
            "Pick vertical stage exceeds the configured "
            f"{max_vertical_stage_mm:.3f} mm limit"
        )

    alignment = build_vision_cartesian_target(
        selection=selection,
        current_values=current,
        xy_offset_mm=xy_offset_mm,
        max_xy_travel_mm=max_xy_travel_mm,
        max_velocity_mm_s=max_velocity_mm_s,
    )
    x_mm = alignment.resolved_plan.x_mm
    y_mm = alignment.resolved_plan.y_mm
    orientation = current[3:]
    velocity = selection.velocity_mm_s
    approach = _complete_target(
        (x_mm, y_mm, z_values[0], *orientation),
        velocity,
        max_velocity_mm_s,
    )
    descend = _complete_target(
        (x_mm, y_mm, z_values[1], *orientation),
        velocity,
        max_velocity_mm_s,
    )
    retract = _complete_target(
        (x_mm, y_mm, z_values[2], *orientation),
        velocity,
        max_velocity_mm_s,
    )
    return PickMotionPlan(
        selection=selection,
        approach=approach,
        descend=descend,
        retract=retract,
        xy_travel_mm=alignment.xy_travel_mm,
    )


def build_named_cartesian_target(
    values: Sequence[float],
    velocity_mm_s: int,
    max_velocity_mm_s: int,
) -> CartesianTargetPlan:
    """Build one configured named Cartesian target such as drop position."""
    return _complete_target(values, velocity_mm_s, max_velocity_mm_s)
