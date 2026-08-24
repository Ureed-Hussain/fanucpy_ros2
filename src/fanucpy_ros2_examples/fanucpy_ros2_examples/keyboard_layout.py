# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Pure keyboard mapping helpers for Cartesian teleoperation."""

from typing import Optional, Tuple


CartesianOffset = Tuple[float, float, float, float, float, float]


def offset_for_key(
    key: str,
    translation_step_mm: float,
    rotation_step_deg: float,
) -> Optional[CartesianOffset]:
    """Return the Cartesian offset represented by one motion key."""
    translation = float(translation_step_mm)
    rotation = float(rotation_step_deg)
    key = key.lower()

    mapping = {
        "w": (0.0, translation, 0.0, 0.0, 0.0, 0.0),
        "s": (0.0, -translation, 0.0, 0.0, 0.0, 0.0),
        "a": (-translation, 0.0, 0.0, 0.0, 0.0, 0.0),
        "d": (translation, 0.0, 0.0, 0.0, 0.0, 0.0),
        "r": (0.0, 0.0, translation, 0.0, 0.0, 0.0),
        "f": (0.0, 0.0, -translation, 0.0, 0.0, 0.0),
        "u": (0.0, 0.0, 0.0, rotation, 0.0, 0.0),
        "o": (0.0, 0.0, 0.0, -rotation, 0.0, 0.0),
        "i": (0.0, 0.0, 0.0, 0.0, rotation, 0.0),
        "k": (0.0, 0.0, 0.0, 0.0, -rotation, 0.0),
        "j": (0.0, 0.0, 0.0, 0.0, 0.0, rotation),
        "l": (0.0, 0.0, 0.0, 0.0, 0.0, -rotation),
    }
    return mapping.get(key)


def bounded_step(current: float, change: float, minimum: float, maximum: float) -> float:
    """Change a teleop step while keeping it inside configured bounds."""
    return min(max(float(current) + float(change), float(minimum)), float(maximum))


def translation_preset_for_key(key: str, maximum: float) -> Optional[float]:
    """Return a safe translation preset for number keys 1 through 5."""
    presets = {
        "1": 1.0,
        "2": 5.0,
        "3": 10.0,
        "4": 25.0,
        "5": 50.0,
    }
    preset = presets.get(key)
    if preset is None:
        return None
    return min(preset, float(maximum))


def velocity_preset_for_key(key: str, maximum: int) -> Optional[int]:
    """Return a percentage of the selected robot's configured velocity limit."""
    percentages = {
        "6": 0.10,
        "7": 0.25,
        "8": 0.50,
        "9": 0.75,
        "0": 1.00,
    }
    percentage = percentages.get(key)
    if percentage is None:
        return None
    return max(1, int(round(int(maximum) * percentage)))


def velocity_increment(maximum: int) -> int:
    """Return a useful fine-adjustment equal to five percent of the limit."""
    return max(1, int(round(int(maximum) * 0.05)))
