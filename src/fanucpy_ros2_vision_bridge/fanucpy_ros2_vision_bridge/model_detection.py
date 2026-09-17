# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Validation for atomic danger_vision model-detection batches."""

from collections import Counter
from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Sequence, Tuple


class DetectionSchemaError(ValueError):
    """Raised when an external detection batch cannot be trusted."""


@dataclass(frozen=True)
class ModelDetection:
    """One normalized conveyor-plane detection."""

    track_id: int
    identity_source: str
    label: str
    confidence: float
    is_danger: bool
    geometry_valid: bool
    projected_center_valid: bool
    projected_u_px: float
    projected_v_px: float
    center_x_m: float
    center_s_m: float
    width_m: float
    length_m: float
    size_profile: str


@dataclass(frozen=True)
class ModelDetectionBatch:
    """One validated atomic detector output."""

    schema_version: int
    source: str
    frame_sequence: int
    stamp_sec: int
    stamp_nanosec: int
    coordinate_space: str
    finish_edge: str
    conveyor_width_m: float
    conveyor_length_m: float
    detections: Tuple[ModelDetection, ...]


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise DetectionSchemaError(f"'{field}' must be an object")
    return value


def _integer(value: Any, field: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DetectionSchemaError(f"'{field}' must be an integer")
    if value < minimum:
        raise DetectionSchemaError(f"'{field}' must be at least {minimum}")
    return value


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DetectionSchemaError(f"'{field}' must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise DetectionSchemaError(f"'{field}' must be finite")
    return number


def _text(value: Any, field: str, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise DetectionSchemaError(f"'{field}' must be a string")
    if len(value) > 256:
        raise DetectionSchemaError(f"'{field}' is unexpectedly long")
    if value and not value.isprintable():
        raise DetectionSchemaError(f"'{field}' contains control characters")
    if not allow_empty and not value.strip():
        raise DetectionSchemaError(f"'{field}' must not be empty")
    return value


def _parse_detection(data: Any) -> ModelDetection:
    item = _mapping(data, "detection")
    track_id = item.get("track_id")
    if isinstance(track_id, bool) or not isinstance(track_id, int):
        raise DetectionSchemaError("'track_id' must be an integer")
    confidence = _finite_number(item.get("confidence"), "confidence")
    if not 0.0 <= confidence <= 1.0:
        raise DetectionSchemaError("'confidence' must be in [0, 1]")
    is_danger = item.get("is_danger")
    if not isinstance(is_danger, bool):
        raise DetectionSchemaError("'is_danger' must be boolean")
    geometry_valid = item.get("geometry_valid", True)
    if not isinstance(geometry_valid, bool):
        raise DetectionSchemaError("'geometry_valid' must be boolean")

    projected_center = item.get("center_projected_px")
    if projected_center is None:
        projected_center_valid = False
        projected_u_px = 0.0
        projected_v_px = 0.0
    else:
        if not isinstance(projected_center, (list, tuple)) or (
            len(projected_center) != 2
        ):
            raise DetectionSchemaError(
                "'center_projected_px' must contain [u, v]"
            )
        projected_center_valid = True
        projected_u_px = _finite_number(
            projected_center[0],
            "center_projected_px[0]",
        )
        projected_v_px = _finite_number(
            projected_center[1],
            "center_projected_px[1]",
        )

    center = _mapping(item.get("center_metric_m"), "center_metric_m")
    width_m = _finite_number(item.get("width_m"), "width_m")
    length_m = _finite_number(item.get("length_m"), "length_m")
    if width_m < 0.0 or length_m < 0.0:
        raise DetectionSchemaError("Object dimensions must not be negative")
    return ModelDetection(
        track_id=track_id,
        identity_source=_text(
            item.get("identity_source", "unspecified"),
            "identity_source",
        ),
        label=_text(item.get("label"), "label"),
        confidence=confidence,
        is_danger=is_danger,
        geometry_valid=geometry_valid,
        projected_center_valid=projected_center_valid,
        projected_u_px=projected_u_px,
        projected_v_px=projected_v_px,
        center_x_m=_finite_number(center.get("x"), "center_metric_m.x"),
        center_s_m=_finite_number(center.get("s"), "center_metric_m.s"),
        width_m=width_m,
        length_m=length_m,
        size_profile=_text(
            item.get("size_profile", ""),
            "size_profile",
            allow_empty=True,
        ),
    )


def parse_model_detection_batch(
    payload: str,
    expected_schema_version: int = 2,
    maximum_detections: int = 100,
    maximum_payload_bytes: int = 1_000_000,
) -> ModelDetectionBatch:
    """Parse and validate one danger_vision JSON detection batch."""
    if not isinstance(payload, str):
        raise DetectionSchemaError("Detection payload must be a string")
    if len(payload.encode("utf-8")) > maximum_payload_bytes:
        raise DetectionSchemaError("Detection payload exceeds the size limit")
    try:
        decoded = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise DetectionSchemaError(
            "Detection payload is not valid JSON"
        ) from exc
    root = _mapping(decoded, "root")

    schema_version = _integer(root.get("schema_version"), "schema_version")
    if schema_version != expected_schema_version:
        raise DetectionSchemaError(
            f"Unsupported detection schema {schema_version}; expected "
            f"{expected_schema_version}"
        )
    coordinate_space = _text(
        root.get("coordinate_space"), "coordinate_space"
    )
    if coordinate_space != "belt_metric":
        raise DetectionSchemaError(
            "Detection coordinate_space must be 'belt_metric'"
        )
    finish_edge = _text(root.get("finish_edge"), "finish_edge")
    if finish_edge not in {"top", "bottom"}:
        raise DetectionSchemaError("finish_edge must be 'top' or 'bottom'")

    width_m = _finite_number(root.get("conveyor_width_m"), "conveyor_width_m")
    length_m = _finite_number(
        root.get("conveyor_length_m"), "conveyor_length_m"
    )
    if width_m <= 0.0 or length_m <= 0.0:
        raise DetectionSchemaError("Conveyor dimensions must be positive")

    stamp = _mapping(root.get("stamp"), "stamp")
    stamp_sec = _integer(stamp.get("sec"), "stamp.sec")
    stamp_nanosec = _integer(stamp.get("nanosec"), "stamp.nanosec")
    if stamp_nanosec >= 1_000_000_000:
        raise DetectionSchemaError("stamp.nanosec must be below one billion")

    raw_detections = root.get("detections")
    if not isinstance(raw_detections, list):
        raise DetectionSchemaError("'detections' must be an array")
    if len(raw_detections) > maximum_detections:
        raise DetectionSchemaError(
            "Detection count exceeds the configured "
            f"{maximum_detections} limit"
        )
    visible_count = _integer(root.get("visible_count"), "visible_count")
    if visible_count != len(raw_detections):
        raise DetectionSchemaError(
            "visible_count does not match the detections array"
        )

    detections = tuple(_parse_detection(item) for item in raw_detections)
    track_ids = [item.track_id for item in detections]
    if len(set(track_ids)) != len(track_ids):
        raise DetectionSchemaError(
            "Detection batch contains duplicate track IDs"
        )
    return ModelDetectionBatch(
        schema_version=schema_version,
        source=_text(root.get("source"), "source"),
        frame_sequence=_integer(
            root.get("frame_sequence"), "frame_sequence"
        ),
        stamp_sec=stamp_sec,
        stamp_nanosec=stamp_nanosec,
        coordinate_space=coordinate_space,
        finish_edge=finish_edge,
        conveyor_width_m=width_m,
        conveyor_length_m=length_m,
        detections=detections,
    )


def label_counts(
    detections: Sequence[ModelDetection],
) -> Tuple[Tuple[str, int], ...]:
    """Count exact detector labels with deterministic ordering."""
    counts = Counter(item.label for item in detections)
    return tuple(sorted(counts.items(), key=lambda item: item[0]))
