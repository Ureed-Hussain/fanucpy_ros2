# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Pure formatting for trusted vision observations supplied to Ollama."""

from collections import Counter, defaultdict
from dataclasses import dataclass
import math
import re
from typing import DefaultDict, Optional, Sequence


_VISION_REQUEST = re.compile(
    r"\b(?:camera|coordinate(?:s)?|detect(?:ed|ion|ions)?|eye[- ]to[- ]hand|"
    r"location(?:s)?|look(?:ing)?|pixel(?:s)?|projected|see|seeing|view|"
    r"visible|vision|where)\b",
    re.IGNORECASE,
)
_EXPLICIT_COORDINATE_REQUEST = re.compile(
    r"\b(?:coordinate(?:s)?|eye[- ]to[- ]hand|location(?:s)?|pixel(?:s)?|"
    r"projected)\b",
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


@dataclass(frozen=True)
class VisionObservation:
    """One normalized object made available for natural-language queries."""

    track_id: int
    label: str
    is_danger: bool
    identity_source: str = ""
    projected_center_valid: bool = False
    projected_u_px: float = 0.0
    projected_v_px: float = 0.0
    eye_to_hand_valid: bool = False
    eye_to_hand_x_mm: float = 0.0
    eye_to_hand_y_mm: float = 0.0


@dataclass(frozen=True)
class AliasedVisionObservation:
    """One observation with deterministic aliases for its current frame."""

    observation: VisionObservation
    object_ordinal: int
    exact_ordinal: int
    family: str
    family_ordinal: int


def is_vision_observation_request(prompt: str) -> bool:
    """Return whether a prompt explicitly asks about visual observations."""
    return bool(_VISION_REQUEST.search(prompt))


def _family_name(label: str) -> str:
    """Return a short human alias without changing the exact label."""
    tokens = set(label.lower().split("-"))
    if "battery" in tokens:
        return "battery"
    if "bottle" in tokens:
        return "bottle"
    return label


def _normalized_words(text: str) -> str:
    """Normalize detector labels and rough user wording for matching."""
    return " ".join(re.sub(r"[^a-z0-9]+", " ", text.lower()).split())


def aliased_vision_observations(
    observations: Sequence[VisionObservation],
) -> Sequence[AliasedVisionObservation]:
    """Assign repeatable aliases in ascending track-ID order."""
    exact_ordinals: DefaultDict[str, int] = defaultdict(int)
    family_ordinals: DefaultDict[str, int] = defaultdict(int)
    aliases = []
    for object_ordinal, observation in enumerate(
        sorted(observations, key=lambda item: (item.track_id, item.label)),
        start=1,
    ):
        exact_ordinals[observation.label] += 1
        family = _family_name(observation.label)
        family_ordinals[family] += 1
        aliases.append(
            AliasedVisionObservation(
                observation=observation,
                object_ordinal=object_ordinal,
                exact_ordinal=exact_ordinals[observation.label],
                family=family,
                family_ordinal=family_ordinals[family],
            )
        )
    return tuple(aliases)


def _ordinal_value(token: str) -> int:
    """Convert a numeric or common spoken ordinal into an integer."""
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


def _coordinate_mode(prompt: str) -> str:
    """Return whether the question asks for projected, eye, or both."""
    asks_projected = bool(re.search(r"\b(?:pixel|pixels|projected)\b", prompt))
    asks_eye = bool(re.search(r"\beye\s+to\s+hand\b", prompt))
    if asks_projected and not asks_eye:
        return "projected"
    if asks_eye and not asks_projected:
        return "eye"
    return "both"


def _spoken_coordinate(
    alias: AliasedVisionObservation,
    mode: str,
    eye_to_hand_frame_id: str,
    eye_to_hand_calibration: str,
) -> str:
    """Format one selected object's trustworthy coordinate measurements."""
    observation = alias.observation
    name = alias.family.replace("-", " ")
    ordinal = (
        alias.family_ordinal
        if alias.family != observation.label
        or alias.family in {"battery", "bottle"}
        else alias.exact_ordinal
    )
    category = "dangerous" if observation.is_danger else "normal"
    prefix = (
        f"{name.capitalize()} {ordinal} ({observation.label}; {category}; "
        f"track ID {observation.track_id})"
    )
    parts = []
    projected_valid = observation.projected_center_valid and all(
        math.isfinite(value)
        for value in (
            observation.projected_u_px,
            observation.projected_v_px,
        )
    )
    eye_valid = observation.eye_to_hand_valid and all(
        math.isfinite(value)
        for value in (
            observation.eye_to_hand_x_mm,
            observation.eye_to_hand_y_mm,
        )
    )
    if mode in {"projected", "both"}:
        if projected_valid:
            parts.append(
                "Its projected center is "
                f"(u={observation.projected_u_px:.2f}, "
                f"v={observation.projected_v_px:.2f}) px."
            )
        else:
            parts.append("Its projected center is unavailable.")
    if mode in {"eye", "both"}:
        if eye_valid:
            frame_id = eye_to_hand_frame_id or "unspecified frame"
            calibration = eye_to_hand_calibration or "unspecified calibration"
            parts.append(
                "Its eye-to-hand position is "
                f"(X={observation.eye_to_hand_x_mm:.3f}, "
                f"Y={observation.eye_to_hand_y_mm:.3f}) mm in {frame_id}, "
                f"using {calibration}."
            )
        else:
            parts.append("Its eye-to-hand position is unavailable.")
    result = f"{prefix}. " + " ".join(parts)
    if mode in {"eye", "both"}:
        result += " This 2D calibration does not provide Z."
    return result


def coordinate_question_answer(
    prompt: str,
    observations: Optional[Sequence[VisionObservation]],
    age_sec: float,
    maximum_age_sec: float,
    eye_to_hand_frame_id: str = "",
    eye_to_hand_calibration: str = "",
) -> Optional[str]:
    """Resolve a read-only coordinate question without model arithmetic."""
    normalized = _normalized_words(prompt)
    explicit_request = bool(_EXPLICIT_COORDINATE_REQUEST.search(prompt))
    where_request = bool(re.search(r"\bwhere\b", normalized))
    if not explicit_request and not where_request:
        return None
    if where_request and not explicit_request and re.search(
        r"\b(?:robot|controller|joint|cartesian|tool)\b",
        normalized,
    ):
        return None
    if observations is None:
        return (
            "I do not have a normalized vision frame yet, so current object "
            "coordinates are unavailable."
        )
    if (
        not math.isfinite(age_sec)
        or age_sec < 0.0
        or age_sec > maximum_age_sec
    ):
        return (
            "The latest vision frame is stale, so I cannot report those "
            "coordinates as current."
        )
    aliases = aliased_vision_observations(observations)
    if not aliases:
        return "No detected objects are visible in the latest vision frame."

    track_match = re.search(r"\btrack(?:\s+id)?\s*#?\s*(\d+)\b", normalized)
    if track_match:
        track_id = int(track_match.group(1))
        selected = [
            alias
            for alias in aliases
            if alias.observation.track_id == track_id
        ]
        if not selected:
            return f"Track ID {track_id} is not visible in the latest frame."
        return _spoken_coordinate(
            selected[0],
            _coordinate_mode(normalized),
            eye_to_hand_frame_id,
            eye_to_hand_calibration,
        )

    object_ordinal = _named_ordinal(normalized, "object")
    if object_ordinal is not None:
        selected = [
            alias
            for alias in aliases
            if alias.object_ordinal == object_ordinal
        ]
        if not selected:
            return (
                f"Object {object_ordinal} is not present; the latest frame "
                f"contains {len(aliases)} objects."
            )
        return _spoken_coordinate(
            selected[0],
            _coordinate_mode(normalized),
            eye_to_hand_frame_id,
            eye_to_hand_calibration,
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
    requested_family: Optional[str] = None
    if exact_name is None:
        if re.search(r"\bbatter(?:y|ies)\b", normalized):
            requested_family = "battery"
        elif re.search(r"\bbottles?\b", normalized):
            requested_family = "bottle"

    if exact_name is not None:
        candidates = [
            alias
            for alias in aliases
            if _normalized_words(alias.observation.label) == exact_name
        ]
        target_name = exact_name
    elif requested_family is not None:
        candidates = [
            alias for alias in aliases if alias.family == requested_family
        ]
        target_name = requested_family
    else:
        single_names = [
            name
            for name in exact_names
            if re.search(
                rf"(?:\b(?:of|for|is)\s+(?:the\s+)?{re.escape(name)}\b|"
                rf"\b{re.escape(name)}\s+{_ORDINAL_TOKEN}\b)",
                normalized,
            )
        ]
        if not single_names:
            if where_request and not explicit_request:
                return None
            if len(aliases) == 1:
                candidates = [aliases[0]]
                target_name = aliases[0].family
            else:
                choices = ", ".join(
                    f"{alias.family} {alias.family_ordinal} "
                    f"(track ID {alias.observation.track_id})"
                    for alias in aliases
                )
                return (
                    "Which current object do you mean? I can use: "
                    f"{choices}."
                )
        else:
            target_name = single_names[0]
            candidates = [
                alias
                for alias in aliases
                if _normalized_words(alias.observation.label) == target_name
            ]

    if not candidates:
        return f"I cannot see a {target_name} in the latest frame."
    ordinal = _named_ordinal(normalized, target_name)
    if ordinal is not None:
        if ordinal > len(candidates):
            return (
                f"I can see only {len(candidates)} {target_name} "
                f"object(s), so {target_name} {ordinal} is unavailable."
            )
        candidates = [candidates[ordinal - 1]]
    elif len(candidates) > 1:
        plural_request = bool(
            re.search(r"\b(?:all|batteries|bottles|objects)\b", normalized)
        )
        if not plural_request:
            choices = ", ".join(
                f"{target_name} {index} (track ID "
                f"{alias.observation.track_id})"
                for index, alias in enumerate(candidates, start=1)
            )
            return f"Which one do you mean? I can use: {choices}."

    mode = _coordinate_mode(normalized)
    answers = [
        _spoken_coordinate(
            alias,
            mode,
            eye_to_hand_frame_id,
            eye_to_hand_calibration,
        )
        for alias in candidates
    ]
    if len(answers) == 1:
        return answers[0]
    return "I found these matching objects:\n- " + "\n- ".join(answers)


def _coordinate_text(
    observation: VisionObservation,
    eye_to_hand_frame_id: str,
    eye_to_hand_calibration: str,
) -> str:
    """Format valid coordinates without exposing numeric placeholders."""
    if observation.projected_center_valid and all(
        math.isfinite(value)
        for value in (
            observation.projected_u_px,
            observation.projected_v_px,
        )
    ):
        projected = (
            f"projected_px=(u={observation.projected_u_px:.2f}, "
            f"v={observation.projected_v_px:.2f}) px"
        )
    else:
        projected = "projected_px=UNAVAILABLE"

    if observation.eye_to_hand_valid and all(
        math.isfinite(value)
        for value in (
            observation.eye_to_hand_x_mm,
            observation.eye_to_hand_y_mm,
        )
    ):
        frame_id = eye_to_hand_frame_id or "unspecified"
        calibration = eye_to_hand_calibration or "unspecified"
        eye_to_hand = (
            f"eye_to_hand[{frame_id}]=(X="
            f"{observation.eye_to_hand_x_mm:.3f}, Y="
            f"{observation.eye_to_hand_y_mm:.3f}) mm; "
            f"calibration={calibration}; Z=NOT_AVAILABLE"
        )
    else:
        eye_to_hand = "eye_to_hand=UNAVAILABLE; Z=NOT_AVAILABLE"
    return f"{projected}; {eye_to_hand}"


def _object_records(
    observations: Sequence[VisionObservation],
    eye_to_hand_frame_id: str,
    eye_to_hand_calibration: str,
) -> str:
    """Create deterministic per-frame aliases and coordinate records."""
    ordered = sorted(
        observations,
        key=lambda item: (item.track_id, item.label),
    )
    exact_ordinals: DefaultDict[str, int] = defaultdict(int)
    family_ordinals: DefaultDict[str, int] = defaultdict(int)
    records = []
    for object_ordinal, observation in enumerate(ordered, start=1):
        exact_ordinals[observation.label] += 1
        exact_alias = (
            f"{observation.label} {exact_ordinals[observation.label]}"
        )
        family = _family_name(observation.label)
        family_ordinals[family] += 1
        aliases = [f"object {object_ordinal}", exact_alias]
        family_alias = f"{family} {family_ordinals[family]}"
        if family_alias != exact_alias:
            aliases.append(family_alias)
        category = "DANGEROUS" if observation.is_danger else "NORMAL"
        records.append(
            "- aliases="
            + ", ".join(f'"{alias}"' for alias in aliases)
            + f"; track_id={observation.track_id}; "
            + f"exact_label={observation.label}; category={category}; "
            + _coordinate_text(
                observation,
                eye_to_hand_frame_id,
                eye_to_hand_calibration,
            )
        )
    if not records:
        return "Visible object records: none"
    return "Visible object records:\n" + "\n".join(records)


def model_vision_context(
    observations: Optional[Sequence[VisionObservation]],
    age_sec: float,
    maximum_age_sec: float,
    source: str = "",
    frame_sequence: int = 0,
    eye_to_hand_frame_id: str = "",
    eye_to_hand_calibration: str = "",
) -> str:
    """Format fresh object counts, aliases, and observation coordinates."""
    if observations is None:
        return (
            "Vision state: UNAVAILABLE\n"
            "No normalized vision batch has been received. Do not claim that "
            "the scene is empty."
        )
    if (
        not math.isfinite(age_sec)
        or age_sec < 0.0
        or age_sec > maximum_age_sec
    ):
        return (
            "Vision state: STALE\n"
            f"Latest normalized vision batch age: {age_sec:.3f} s. "
            "Do not report these objects as currently visible."
        )
    danger_counts = Counter(
        item.label for item in observations if item.is_danger
    )
    normal_counts = Counter(
        item.label for item in observations if not item.is_danger
    )

    def formatted_counts(title: str, counts: Counter) -> str:
        """Format one category without changing detector labels."""
        if not counts:
            return f"Visible {title} exact-label counts: none"
        return f"Visible {title} exact-label counts:\n" + "\n".join(
            f"- {label}: {count}" for label, count in sorted(counts.items())
        )

    count_context = (
        formatted_counts("DANGEROUS", danger_counts)
        + "\n"
        + formatted_counts("NORMAL", normal_counts)
    )
    object_context = _object_records(
        observations,
        eye_to_hand_frame_id,
        eye_to_hand_calibration,
    )
    return (
        "Vision state: FRESH\n"
        f"Vision source: {source or 'unspecified'}\n"
        f"Vision frame sequence: {int(frame_sequence)}\n"
        f"{count_context}\n"
        f"{object_context}\n"
        "Object ordinals are recomputed for each fresh frame in ascending "
        "track-ID order; use track_id for unambiguous selection in that "
        "frame.\n"
        "Coordinates are observations only and must not be converted into a "
        "motion action. Eye-to-hand calibration supplies X and Y only; Z is "
        "not available."
    )
