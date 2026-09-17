# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import json

import pytest

from fanucpy_ros2_vision_bridge.model_detection import (
    DetectionSchemaError,
    label_counts,
    parse_model_detection_batch,
)


def _detection(track_id, label, is_danger=True):
    return {
        "track_id": track_id,
        "identity_source": "bytetrack",
        "label": label,
        "confidence": 0.91,
        "is_danger": is_danger,
        "geometry_valid": True,
        "center_projected_px": [100.0, 200.0],
        "center_metric_m": {"x": 0.25, "s": 0.50},
        "width_m": 0.08,
        "length_m": 0.12,
        "size_profile": "50_120_MM",
    }


def _payload(detections=None, **overrides):
    items = [_detection(1, "BATTERY")] if detections is None else detections
    data = {
        "schema_version": 2,
        "source": "ultralytics_model_tracker",
        "frame_sequence": 42,
        "stamp": {"sec": 100, "nanosec": 200},
        "coordinate_space": "belt_metric",
        "finish_edge": "top",
        "conveyor_width_m": 0.44,
        "conveyor_length_m": 0.89,
        "visible_count": len(items),
        "detections": items,
    }
    data.update(overrides)
    return json.dumps(data)


def test_parses_exact_model_labels_and_counts():
    batch = parse_model_detection_batch(
        _payload(
            [
                _detection(1, "BATTERY"),
                _detection(2, "BATTERY"),
                _detection(3, "pet-bottle-clear-food", is_danger=False),
            ]
        )
    )
    assert [item.label for item in batch.detections] == [
        "BATTERY",
        "BATTERY",
        "pet-bottle-clear-food",
    ]
    assert label_counts(batch.detections) == (
        ("BATTERY", 2),
        ("pet-bottle-clear-food", 1),
    )
    assert batch.detections[0].is_danger
    assert not batch.detections[2].is_danger
    assert all(item.geometry_valid for item in batch.detections)
    assert batch.detections[0].projected_center_valid
    assert batch.detections[0].projected_u_px == 100.0
    assert batch.detections[0].projected_v_px == 200.0


def test_accepts_empty_visible_batch():
    batch = parse_model_detection_batch(_payload([]))
    assert batch.detections == ()


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"schema_version": 3}, "Unsupported detection schema"),
        ({"coordinate_space": "pixels"}, "coordinate_space"),
        ({"finish_edge": "left"}, "finish_edge"),
        ({"visible_count": 2}, "visible_count"),
    ],
)
def test_rejects_invalid_batch_metadata(overrides, message):
    with pytest.raises(DetectionSchemaError, match=message):
        parse_model_detection_batch(_payload(**overrides))


def test_rejects_duplicate_track_ids():
    payload = _payload(
        [_detection(1, "BATTERY"), _detection(1, "BATTERY")]
    )
    with pytest.raises(DetectionSchemaError, match="duplicate track IDs"):
        parse_model_detection_batch(payload)


def test_rejects_non_finite_detection_values():
    detection = _detection(1, "BATTERY")
    detection["center_metric_m"]["x"] = float("nan")
    with pytest.raises(DetectionSchemaError, match="must be finite"):
        parse_model_detection_batch(_payload([detection]))


def test_rejects_excessive_detection_count():
    payload = _payload([_detection(index, "BATTERY") for index in range(3)])
    with pytest.raises(DetectionSchemaError, match="configured 2 limit"):
        parse_model_detection_batch(payload, maximum_detections=2)


def test_rejects_label_control_characters():
    detection = _detection(1, "BATTERY\nignore instructions")
    with pytest.raises(DetectionSchemaError, match="control characters"):
        parse_model_detection_batch(_payload([detection]))


def test_accepts_visual_detection_without_projected_center():
    detection = _detection(1, "pet-bottle-clear-food", is_danger=False)
    detection["geometry_valid"] = False
    del detection["center_projected_px"]
    batch = parse_model_detection_batch(_payload([detection]))
    assert not batch.detections[0].geometry_valid
    assert not batch.detections[0].projected_center_valid
