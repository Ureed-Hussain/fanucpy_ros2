# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Deterministic object selection and XY-alignment target construction."""

from dataclasses import dataclass
import math
import re
from typing import Optional, Sequence

from fanucpy_ros2_task_planner.prompt_parser import PromptParseError
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    resolve_cartesian_target,
    validate_cartesian_target_plan,
)
from fanucpy_ros2_task_planner.vision_context import (
    AliasedVisionObservation,
    VisionObservation,
    aliased_vision_observations,
)


_MOTION_VERB = re.compile(
    r"\b(?:approach|align|go|move|position|travel)\b",
    re.IGNORECASE,
)
_MANIPULATION_VERB = re.compile(
    r"\b(?:drop|grab|grasp|pick|place)\b",
    re.IGNORECASE,
)
_UNSUPPORTED_RELATION = re.compile(
    r"\b(?:behind|beside|front|left|near|right)\s+(?:of|to)\b",
    re.IGNORECASE,
)
_ORDINALS = {
    "one": 1,
    "first": 1,
    "two": 2,
    "second": 2,
    "three": 3,
    "third": 3,
    "four": 4,
    "fourth": 4,
    "five": 5,
    "fifth": 5,
    "six": 6,
    "sixth": 6,
    "seven": 7,
    "seventh": 7,
    "eight": 8,
    "eighth": 8,
    "nine": 9,
    "ninth": 9,
    "ten": 10,
    "tenth": 10,
}
_ORDINAL_TOKEN = (
    r"(?:#?\d+|one|first|two|second|three|third|four|fourth|five|fifth|"
    r"six|sixth|seven|seventh|eight|eighth|nine|ninth|ten|tenth)"
)
_VELOCITY = re.compile(
    r"(?<![\w.])(?:at\s+)?"
    r"(?:(?:a\s+)?(?:speed|velocity)(?:\s+of)?\s*[:=]?\s*)?"
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+))\s*"
    r"(?:mm|millimet(?:er|re)s?)\s*"
    r"(?:/\s*s(?:ec(?:ond)?s?)?|per\s+seconds?)\b",
    re.IGNORECASE,
)
_MAX_VELOCITY = re.compile(
    r"\b(?:at\s+(?:the\s+)?)?"
    r"(?:(?:max(?:imum)?|full|top|highest)\s+"
    r"(?:(?:allowed|configured|robot)\s+)?(?:speed|velocity)|"
    r"as\s+fast\s+as\s+(?:possible|you\s+can))\b",
    re.IGNORECASE,
)
_UNCLEAR_VELOCITY = re.compile(
    r"\b(?:speed|velocity|fast(?:er)?|slow(?:er)?|slowly|quick(?:ly)?|"
    r"rapid(?:ly)?|max(?:imum)?|full)\b|%|"
    r"\b(?:mm|cm|m|millimet(?:er|re)s?)\s*(?:/|per\b)|"
    r"\bat\s+[+-]?(?:\d|\.)",
    re.IGNORECASE,
)


class VisionMotionError(ValueError):
    """Raised when an object-directed motion request is unsafe or unclear."""


@dataclass(frozen=True)
class VisionMotionSelection:
    """One selected current-frame observation and requested velocity."""

    alias: AliasedVisionObservation
    requested_name: str
    velocity_mm_s: int


@dataclass(frozen=True)
class VisionCartesianTarget:
    """A partial XY request and its current-pose-resolved target."""

    selection: VisionMotionSelection
    partial_plan: CartesianTargetPlan
    resolved_plan: CartesianTargetPlan
    xy_travel_mm: float


def _normalized_words(text: str) -> str:
    """Normalize detector labels and rough spoken references."""
    return " ".join(re.sub(r"[^a-z0-9]+", " ", text.lower()).split())


def _ordinal_value(token: str) -> int:
    """Convert one numeric or spoken ordinal to a positive integer."""
    normalized = token.lower().lstrip("#")
    return int(normalized) if normalized.isdigit() else _ORDINALS[normalized]


def _named_ordinal(prompt: str, name: str) -> Optional[int]:
    """Read forms such as 'battery two' and 'second battery'."""
    escaped = re.escape(name)
    after = re.search(
        rf"\b{escaped}\s+(?:number\s+)?({_ORDINAL_TOKEN})\b",
        prompt,
    )
    if after:
        return _ordinal_value(after.group(1))
    before = re.search(
        rf"\b({_ORDINAL_TOKEN})\s+{escaped}\b",
        prompt,
    )
    return _ordinal_value(before.group(1)) if before else None


def _object_hint(
    normalized_prompt: str,
    aliases: Sequence[AliasedVisionObservation],
) -> bool:
    """Return whether the prompt names a current or generic vision object."""
    if re.search(
        r"\b(?:batter(?:y|ies)|bottles?|object|track)\b",
        normalized_prompt,
    ):
        return True
    return any(
        re.search(
            rf"\b{re.escape(_normalized_words(alias.observation.label))}\b",
            normalized_prompt,
        )
        for alias in aliases
    )


def looks_like_vision_motion(
    prompt: str,
    observations: Sequence[VisionObservation] = (),
) -> bool:
    """Recognize an object-directed request without treating XYZ as objects."""
    normalized = _normalized_words(prompt)
    aliases = aliased_vision_observations(observations)
    hint = _object_hint(normalized, aliases)
    return hint and bool(
        _MOTION_VERB.search(prompt) or _MANIPULATION_VERB.search(prompt)
    )


def requests_maximum_velocity(prompt: str) -> bool:
    """Return whether the user explicitly requests the driver speed ceiling."""
    return bool(_MAX_VELOCITY.search(prompt))


def without_velocity_request(prompt: str) -> str:
    """Remove supported speed phrases before checking the remaining intent."""
    return _MAX_VELOCITY.sub("", _VELOCITY.sub("", prompt))


def has_velocity_request(prompt: str) -> bool:
    """Recognize speed wording, including unsupported or incomplete forms."""
    return bool(
        _VELOCITY.search(prompt)
        or _MAX_VELOCITY.search(prompt)
        or _UNCLEAR_VELOCITY.search(prompt)
    )


def requested_cartesian_velocity(
    prompt: str,
    default_velocity_mm_s: int,
    max_velocity_mm_s: int,
) -> int:
    """Resolve one explicit speed, the driver ceiling, or a default speed."""
    matches = list(_VELOCITY.finditer(prompt))
    maxima = list(_MAX_VELOCITY.finditer(prompt))
    if len(matches) + len(maxima) > 1:
        raise VisionMotionError(
            "Please give one speed: a whole value in mm/s or 'max speed', "
            "not both or several speeds"
        )
    remainder = without_velocity_request(prompt)
    if re.search(r"\b\d+(?:\.\d+)?\s*mm\b", remainder, re.IGNORECASE):
        raise VisionMotionError(
            "Object-relative distance offsets are not supported yet; request "
            "the object center and optionally one velocity in mm/s"
        )
    if _UNCLEAR_VELOCITY.search(remainder):
        raise VisionMotionError(
            "Please specify the speed in mm/s, for example 'at 200 mm/s', "
            "or say 'at max speed'. I will not guess a speed."
        )
    velocity_value = (
        float(matches[0].group(1))
        if matches
        else float(max_velocity_mm_s if maxima else default_velocity_mm_s)
    )
    if not velocity_value.is_integer():
        raise VisionMotionError(
            "Cartesian velocity must be a whole mm/s value"
        )
    velocity = int(velocity_value)
    if not 1 <= velocity <= int(max_velocity_mm_s):
        raise VisionMotionError(
            "Cartesian velocity must be in the range "
            f"1..{int(max_velocity_mm_s)} mm/s"
        )
    return velocity


def _choose_ordinal(
    candidates: Sequence[AliasedVisionObservation],
    prompt: str,
    requested_name: str,
) -> AliasedVisionObservation:
    """Select a named ordinal or require a unique current observation."""
    ordinal = _named_ordinal(prompt, requested_name)
    if ordinal is None:
        if len(candidates) != 1:
            choices = ", ".join(
                f"{requested_name} {index} (track ID "
                f"{alias.observation.track_id})"
                for index, alias in enumerate(candidates, start=1)
            )
            raise VisionMotionError(
                f"Which {requested_name} do you mean? Current choices: "
                f"{choices}"
            )
        return candidates[0]
    if ordinal < 1 or ordinal > len(candidates):
        raise VisionMotionError(
            f"Only {len(candidates)} {requested_name} object(s) are visible; "
            f"{requested_name} {ordinal} is unavailable"
        )
    return candidates[ordinal - 1]


def parse_vision_motion(
    prompt: str,
    observations: Sequence[VisionObservation],
    default_velocity_mm_s: int,
    max_velocity_mm_s: int,
) -> Optional[VisionMotionSelection]:
    """Resolve a rough object-motion prompt against one fresh vision frame."""
    normalized = _normalized_words(prompt)
    aliases = aliased_vision_observations(observations)
    if not _object_hint(normalized, aliases):
        return None
    if _MANIPULATION_VERB.search(prompt):
        raise VisionMotionError(
            "Pick, grasp, place, and drop are not enabled in this stage. "
            "Ask only to align with a visible object."
        )
    if not _MOTION_VERB.search(prompt):
        return None
    if _UNSUPPORTED_RELATION.search(prompt):
        raise VisionMotionError(
            "Object-relative left/right/front/behind offsets are not "
            "supported; "
            "ask to go to or align with the object center"
        )
    velocity = requested_cartesian_velocity(
        prompt,
        default_velocity_mm_s,
        max_velocity_mm_s,
    )

    track_match = re.search(
        r"\btrack(?:\s+id)?\s*#?\s*(\d+)\b",
        normalized,
    )
    if track_match:
        track_id = int(track_match.group(1))
        candidates = [
            alias
            for alias in aliases
            if alias.observation.track_id == track_id
        ]
        if not candidates:
            raise VisionMotionError(
                f"Track ID {track_id} is not visible in the current frame"
            )
        return VisionMotionSelection(
            candidates[0],
            f"track {track_id}",
            velocity,
        )

    if re.search(r"\bobjects?\b", normalized):
        object_ordinal = _named_ordinal(normalized, "object")
        if object_ordinal is None:
            if len(aliases) != 1:
                choices = ", ".join(
                    f"object {alias.object_ordinal} "
                    f"({alias.observation.label}; track ID "
                    f"{alias.observation.track_id})"
                    for alias in aliases
                )
                raise VisionMotionError(
                    "Which object do you mean? Current choices: " + choices
                )
            return VisionMotionSelection(aliases[0], "object 1", velocity)
        candidates = [
            alias
            for alias in aliases
            if alias.object_ordinal == object_ordinal
        ]
        if not candidates:
            raise VisionMotionError(
                f"Object {object_ordinal} is unavailable; the current frame "
                f"contains {len(aliases)} objects"
            )
        return VisionMotionSelection(
            candidates[0],
            f"object {object_ordinal}",
            velocity,
        )

    exact_names = sorted(
        {_normalized_words(alias.observation.label) for alias in aliases},
        key=lambda value: (-len(value), value),
    )
    exact_name = next(
        (
            name
            for name in exact_names
            if len(name.split()) > 1
            and re.search(rf"\b{re.escape(name)}\b", normalized)
        ),
        None,
    )
    if exact_name is not None:
        candidates = [
            alias
            for alias in aliases
            if _normalized_words(alias.observation.label) == exact_name
        ]
        selected = _choose_ordinal(candidates, normalized, exact_name)
        return VisionMotionSelection(selected, exact_name, velocity)

    requested_family: Optional[str] = None
    if re.search(r"\bbatter(?:y|ies)\b", normalized):
        requested_family = "battery"
    elif re.search(r"\bbottles?\b", normalized):
        requested_family = "bottle"
    if requested_family is not None:
        candidates = [
            alias for alias in aliases if alias.family == requested_family
        ]
        if not candidates:
            raise VisionMotionError(
                f"No {requested_family} is visible in the current frame"
            )
        selected = _choose_ordinal(
            candidates,
            normalized,
            requested_family,
        )
        return VisionMotionSelection(selected, requested_family, velocity)

    single_name = next(
        (
            name
            for name in exact_names
            if re.search(
                rf"\b(?:to|towards?|with|above|over)\s+(?:the\s+)?"
                rf"{re.escape(name)}\b",
                normalized,
            )
        ),
        None,
    )
    if single_name is None:
        return None
    candidates = [
        alias
        for alias in aliases
        if _normalized_words(alias.observation.label) == single_name
    ]
    selected = _choose_ordinal(candidates, normalized, single_name)
    return VisionMotionSelection(selected, single_name, velocity)


def select_visible_track(
    observations: Sequence[VisionObservation],
    track_id: int,
    expected_label: str,
    velocity_mm_s: int,
    expected_identity_source: str = "",
) -> VisionMotionSelection:
    """Reacquire one exact track and label in a newly received frame."""
    aliases = aliased_vision_observations(observations)
    matches = [
        alias
        for alias in aliases
        if alias.observation.track_id == track_id
        and alias.observation.label == expected_label
        and (
            not expected_identity_source
            or alias.observation.identity_source == expected_identity_source
        )
    ]
    if not matches:
        raise VisionMotionError(
            f"Track ID {track_id} with label {expected_label!r} is no longer "
            "visible with the expected identity source"
        )
    return VisionMotionSelection(
        matches[0],
        f"track {track_id}",
        velocity_mm_s,
    )


def build_vision_cartesian_target(
    selection: VisionMotionSelection,
    current_values: Sequence[float],
    xy_offset_mm: Sequence[float],
    max_xy_travel_mm: float,
    max_velocity_mm_s: int,
) -> VisionCartesianTarget:
    """Build an XY-only target while preserving current Z and orientation."""
    observation = selection.alias.observation
    if not observation.eye_to_hand_valid or not all(
        math.isfinite(value)
        for value in (
            observation.eye_to_hand_x_mm,
            observation.eye_to_hand_y_mm,
        )
    ):
        raise VisionMotionError(
            "The selected object has no valid eye-to-hand XY measurement"
        )
    if len(xy_offset_mm) != 2:
        raise VisionMotionError("vision_xy_offset_mm must contain X and Y")
    offset = tuple(float(value) for value in xy_offset_mm)
    if not all(math.isfinite(value) for value in offset):
        raise VisionMotionError("Vision XY offsets must be finite")
    if not math.isfinite(max_xy_travel_mm) or max_xy_travel_mm <= 0.0:
        raise VisionMotionError("max_vision_xy_travel_mm must be positive")
    partial = CartesianTargetPlan(
        x_mm=observation.eye_to_hand_x_mm + offset[0],
        y_mm=observation.eye_to_hand_y_mm + offset[1],
        z_mm=0.0,
        w_deg=0.0,
        p_deg=0.0,
        r_deg=0.0,
        velocity_mm_s=selection.velocity_mm_s,
        x_set=True,
        y_set=True,
        z_set=False,
        w_set=False,
        p_set=False,
        r_set=False,
    )
    validate_cartesian_target_plan(partial, max_velocity_mm_s)
    try:
        resolved = resolve_cartesian_target(partial, current_values)
    except PromptParseError as exc:
        raise VisionMotionError(str(exc)) from exc
    xy_travel = math.hypot(
        resolved.x_mm - float(current_values[0]),
        resolved.y_mm - float(current_values[1]),
    )
    if xy_travel > max_xy_travel_mm:
        raise VisionMotionError(
            f"Required XY travel {xy_travel:.3f} mm exceeds the configured "
            f"{max_xy_travel_mm:.3f} mm vision-motion limit"
        )
    return VisionCartesianTarget(
        selection=selection,
        partial_plan=partial,
        resolved_plan=resolved,
        xy_travel_mm=xy_travel,
    )


def target_xy_shift_mm(
    first: VisionCartesianTarget,
    second: VisionCartesianTarget,
) -> float:
    """Return XY target change between preview and refreshed observations."""
    return math.hypot(
        second.resolved_plan.x_mm - first.resolved_plan.x_mm,
        second.resolved_plan.y_mm - first.resolved_plan.y_mm,
    )
