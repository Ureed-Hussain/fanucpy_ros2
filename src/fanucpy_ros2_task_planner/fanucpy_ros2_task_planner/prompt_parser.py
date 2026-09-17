# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Deterministically convert a small prompt grammar into Cartesian jogs."""

from dataclasses import dataclass
import re
from typing import Dict, List, Match, Optional, Tuple


CartesianOffset = Tuple[float, float, float, float, float, float]

_NUMBER = r"(?:\d+(?:\.\d*)?|\.\d+)"
_DIRECTION = r"left|right|up|down|forward|forwards|backward|backwards"
_TRANSLATION_UNIT = (
    r"mm|millimeters?|millimetres?|cm|centimeters?|centimetres?"
)
_ROTATION_AXIS = r"roll|pitch|yaw|w|p|r|x|y|z"
_ROTATION_SIGN = r"positive|negative|clockwise|counterclockwise"
_ROTATION_UNIT = r"deg|degree|degrees"

_SPEED_PATTERN = re.compile(
    rf"\b(?:at(?:\s+(?:a\s+)?speed(?:\s+of)?)?|"
    rf"with\s+(?:a\s+)?speed(?:\s+of)?|speed(?:\s+of)?)\s+"
    rf"(?P<amount>{_NUMBER})\s*"
    r"(?P<unit>mm/s|mmps|millimeters?\s+per\s+second|"
    r"millimetres?\s+per\s+second)\b"
)

_TRANSLATION_PATTERN = re.compile(
    rf"^(?:please\s+)?"
    rf"(?:(?:move|go|jog)(?:\s+the\s+robot)?(?:\s+to)?\s+)?"
    rf"(?:the\s+)?(?P<direction>{_DIRECTION})"
    rf"(?:\s+by)?"
    rf"(?:\s+(?P<amount>{_NUMBER})"
    rf"(?:\s*(?P<unit>{_TRANSLATION_UNIT}))?)?$"
)

_ROTATION_AXIS_FIRST_PATTERN = re.compile(
    rf"^(?:please\s+)?(?:rotate|turn)(?:\s+the\s+robot)?"
    rf"(?:\s+(?:around|about))?\s+"
    rf"(?P<axis>{_ROTATION_AXIS})(?:\s+axis)?"
    rf"(?:\s+(?P<sign>{_ROTATION_SIGN}))?"
    rf"(?:\s+by)?"
    rf"(?:\s+(?P<amount>{_NUMBER})"
    rf"(?:\s*(?P<unit>{_ROTATION_UNIT}))?)?$"
)

_ROTATION_SIGN_FIRST_PATTERN = re.compile(
    rf"^(?:please\s+)?(?:rotate|turn)(?:\s+the\s+robot)?\s+"
    rf"(?P<sign>clockwise|counterclockwise)"
    rf"(?:\s+by)?"
    rf"(?:\s+(?P<amount>{_NUMBER})"
    rf"(?:\s*(?P<unit>{_ROTATION_UNIT}))?)?"
    rf"(?:\s+(?:around|about)\s+(?:the\s+)?"
    rf"(?P<axis>{_ROTATION_AXIS})(?:\s+axis)?)?$"
)

_CLAUSE_SEPARATOR = re.compile(r"\s*(?:,|\band\b|\bthen\b)\s*")

_DIRECTION_COMPONENTS: Dict[str, Tuple[int, float]] = {
    "left": (0, -1.0),
    "right": (0, 1.0),
    "forward": (1, 1.0),
    "forwards": (1, 1.0),
    "backward": (1, -1.0),
    "backwards": (1, -1.0),
    "up": (2, 1.0),
    "down": (2, -1.0),
}

_ROTATION_COMPONENTS: Dict[str, int] = {
    "roll": 3,
    "w": 3,
    "x": 3,
    "pitch": 4,
    "p": 4,
    "y": 4,
    "yaw": 5,
    "r": 5,
    "z": 5,
}


class PromptParseError(ValueError):
    """Raised when a prompt is outside the deliberately small grammar."""


@dataclass(frozen=True)
class CartesianJogPlan:
    """One validated relative Cartesian jog produced from a prompt."""

    delta_x_mm: float
    delta_y_mm: float
    delta_z_mm: float
    delta_w_deg: float
    delta_p_deg: float
    delta_r_deg: float
    velocity_mm_s: int = 0

    @property
    def offset(self) -> CartesianOffset:
        """Return offsets in the JogCartesian action field order."""
        return (
            self.delta_x_mm,
            self.delta_y_mm,
            self.delta_z_mm,
            self.delta_w_deg,
            self.delta_p_deg,
            self.delta_r_deg,
        )


def _normalize(prompt: str) -> str:
    normalized = prompt.strip().lower().replace("°", " deg")
    normalized = normalized.replace("counter-clockwise", "counterclockwise")
    normalized = normalized.replace("counter clockwise", "counterclockwise")
    normalized = normalized.rstrip(".!?")
    return re.sub(r"\s+", " ", normalized)


def _translation_amount(match: Match[str], default_mm: float) -> float:
    amount_text = match.group("amount")
    if amount_text is None:
        return default_mm
    amount = float(amount_text)
    unit = match.group("unit") or "mm"
    centimetre_units = {
        "cm",
        "centimeter",
        "centimeters",
        "centimetre",
        "centimetres",
    }
    if unit in centimetre_units:
        amount *= 10.0
    return amount


def _rotation_amount(match: Match[str], default_deg: float) -> float:
    amount_text = match.group("amount")
    if amount_text is None:
        return default_deg
    return float(amount_text)


def _rotation_sign(sign: Optional[str]) -> float:
    if sign in {"negative", "clockwise"}:
        return -1.0
    return 1.0


def validate_jog_plan(
    plan: CartesianJogPlan,
    max_translation_step_mm: float,
    max_rotation_step_deg: float,
    max_cartesian_velocity_mm_s: int,
) -> CartesianJogPlan:
    """Check one plan against the supplied per-axis driver-style limits."""
    if max_translation_step_mm <= 0.0:
        raise PromptParseError("Maximum translation must be greater than zero")
    if max_rotation_step_deg <= 0.0:
        raise PromptParseError("Maximum rotation must be greater than zero")
    if max_cartesian_velocity_mm_s <= 0:
        raise PromptParseError(
            "Maximum Cartesian velocity must be greater than zero"
        )
    if all(value == 0.0 for value in plan.offset):
        raise PromptParseError(
            "The prompt does not contain a non-zero movement"
        )
    if any(abs(value) > max_translation_step_mm for value in plan.offset[:3]):
        raise PromptParseError(
            "Translation exceeds the configured per-axis limit of "
            f"{max_translation_step_mm:g} mm"
        )
    if any(abs(value) > max_rotation_step_deg for value in plan.offset[3:]):
        raise PromptParseError(
            "Rotation exceeds the configured per-axis limit of "
            f"{max_rotation_step_deg:g} degrees"
        )
    if not 0 <= plan.velocity_mm_s <= max_cartesian_velocity_mm_s:
        raise PromptParseError(
            "Velocity must be zero for the driver default or within 1.."
            f"{max_cartesian_velocity_mm_s} mm/s"
        )
    return plan


class PromptParser:
    """Parse only documented Cartesian commands; never guess unknown intent."""

    def __init__(
        self,
        default_translation_mm: float = 10.0,
        default_rotation_deg: float = 1.0,
        max_translation_step_mm: float = 50.0,
        max_rotation_step_deg: float = 2.0,
        max_cartesian_velocity_mm_s: int = 2000,
    ) -> None:
        self.default_translation_mm = float(default_translation_mm)
        self.default_rotation_deg = float(default_rotation_deg)
        self.max_translation_step_mm = float(max_translation_step_mm)
        self.max_rotation_step_deg = float(max_rotation_step_deg)
        self.max_cartesian_velocity_mm_s = int(max_cartesian_velocity_mm_s)

        if not (
            0.0
            < self.default_translation_mm
            <= self.max_translation_step_mm
        ):
            raise ValueError(
                "Default translation is outside the configured limits"
            )
        if not (
            0.0 < self.default_rotation_deg <= self.max_rotation_step_deg
        ):
            raise ValueError(
                "Default rotation is outside the configured limits"
            )
        if self.max_cartesian_velocity_mm_s <= 0:
            raise ValueError(
                "Maximum Cartesian velocity must be greater than zero"
            )

    def parse(self, prompt: str) -> CartesianJogPlan:
        """Return one Cartesian plan or reject the complete prompt."""
        normalized = _normalize(prompt)
        if not normalized:
            raise PromptParseError("The prompt is empty")

        velocity, movement_text = self._extract_velocity(normalized)
        clauses = [
            clause.strip()
            for clause in _CLAUSE_SEPARATOR.split(movement_text)
            if clause.strip()
        ]
        if not clauses:
            raise PromptParseError("The prompt does not contain a movement")

        values: List[float] = [0.0] * 6
        used_components = set()
        for clause in clauses:
            component, value = self._parse_clause(clause)
            if component in used_components:
                component_name = ("X", "Y", "Z", "W", "P", "R")[component]
                raise PromptParseError(
                    f"The prompt commands {component_name} more than once"
                )
            used_components.add(component)
            values[component] = value

        plan = CartesianJogPlan(*values, velocity_mm_s=velocity)
        return validate_jog_plan(
            plan,
            self.max_translation_step_mm,
            self.max_rotation_step_deg,
            self.max_cartesian_velocity_mm_s,
        )

    def _extract_velocity(self, prompt: str) -> Tuple[int, str]:
        matches = list(_SPEED_PATTERN.finditer(prompt))
        if len(matches) > 1:
            raise PromptParseError("Specify Cartesian velocity only once")
        if not matches:
            return 0, prompt

        amount = float(matches[0].group("amount"))
        if not amount.is_integer():
            raise PromptParseError("Cartesian velocity must be a whole number")
        velocity = int(amount)
        if not 1 <= velocity <= self.max_cartesian_velocity_mm_s:
            raise PromptParseError(
                "Cartesian velocity must be within 1.."
                f"{self.max_cartesian_velocity_mm_s} mm/s"
            )

        movement_text = _SPEED_PATTERN.sub("", prompt, count=1).strip(" ,")
        movement_text = re.sub(r"(?:\band\b|\bthen\b)\s*$", "", movement_text)
        return velocity, movement_text.strip(" ,")

    def _parse_clause(self, clause: str) -> Tuple[int, float]:
        translation = _TRANSLATION_PATTERN.fullmatch(clause)
        if translation is not None:
            direction = translation.group("direction")
            component, sign = _DIRECTION_COMPONENTS[direction]
            amount = _translation_amount(
                translation,
                self.default_translation_mm,
            )
            if amount <= 0.0:
                raise PromptParseError("Translation must be greater than zero")
            return component, sign * amount

        rotation = _ROTATION_AXIS_FIRST_PATTERN.fullmatch(clause)
        if rotation is None:
            rotation = _ROTATION_SIGN_FIRST_PATTERN.fullmatch(clause)
        if rotation is not None:
            axis = rotation.group("axis") or "yaw"
            component = _ROTATION_COMPONENTS[axis]
            amount = _rotation_amount(rotation, self.default_rotation_deg)
            if amount <= 0.0:
                raise PromptParseError("Rotation must be greater than zero")
            return component, _rotation_sign(rotation.group("sign")) * amount

        raise PromptParseError(
            f"Unsupported or ambiguous movement clause: '{clause}'"
        )


def format_plan(plan: CartesianJogPlan, frame_id: str) -> str:
    """Create a stable operator-facing preview of a parsed plan."""
    velocity = (
        f"{plan.velocity_mm_s} mm/s"
        if plan.velocity_mm_s > 0
        else "driver default"
    )
    values = ", ".join(f"{value:+.3f}" for value in plan.offset)
    return (
        f"Frame: {frame_id}\n"
        f"Offset [dX dY dZ dW dP dR]: [{values}] [mm, deg]\n"
        f"Velocity: {velocity}"
    )
