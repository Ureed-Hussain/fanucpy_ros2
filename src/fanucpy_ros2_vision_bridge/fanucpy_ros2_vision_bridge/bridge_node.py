#!/usr/bin/env python3
# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Bridge validated danger_vision JSON into typed fanucpy_ros2 messages."""

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from fanucpy_ros2_interfaces.msg import (
    VisionDetection,
    VisionDetectionArray,
)

from .affine_transform import EyeToHandAffine
from .model_detection import (
    DetectionSchemaError,
    ModelDetectionBatch,
    label_counts,
    parse_model_detection_batch,
)


class FanucpyVisionBridge(Node):
    """Normalize one external detector batch without commanding the robot."""

    def __init__(self) -> None:
        super().__init__("fanucpy_vision_bridge")
        self.declare_parameter(
            "input_topic", "/danger/model/observations"
        )
        self.declare_parameter(
            "output_topic", "/fanuc/vision/detections"
        )
        self.declare_parameter("status_topic", "/fanuc/vision/status")
        self.declare_parameter("output_frame_id", "conveyor_m")
        self.declare_parameter("expected_schema_version", 2)
        self.declare_parameter("maximum_detections", 100)
        self.declare_parameter("maximum_payload_bytes", 1_000_000)
        self.declare_parameter("eye_to_hand_enabled", True)
        self.declare_parameter("eye_to_hand_frame_id", "fanuc_world")
        self.declare_parameter(
            "eye_to_hand_calibration",
            "phase_two_active_affine",
        )
        self.declare_parameter(
            "eye_to_hand_matrix",
            [-0.01645279, -1.00536662, -1.00400079, -0.00456943],
        )
        self.declare_parameter(
            "eye_to_hand_translation_mm",
            [-170.43887235, 1309.18504954],
        )

        self.input_topic = str(self.get_parameter("input_topic").value).strip()
        self.output_topic = str(
            self.get_parameter("output_topic").value
        ).strip()
        self.status_topic = str(
            self.get_parameter("status_topic").value
        ).strip()
        self.output_frame_id = str(
            self.get_parameter("output_frame_id").value
        ).strip()
        self.expected_schema_version = int(
            self.get_parameter("expected_schema_version").value
        )
        self.maximum_detections = int(
            self.get_parameter("maximum_detections").value
        )
        self.maximum_payload_bytes = int(
            self.get_parameter("maximum_payload_bytes").value
        )
        self.eye_to_hand_enabled = bool(
            self.get_parameter("eye_to_hand_enabled").value
        )
        self.eye_to_hand_frame_id = str(
            self.get_parameter("eye_to_hand_frame_id").value
        ).strip()
        self.eye_to_hand_calibration = str(
            self.get_parameter("eye_to_hand_calibration").value
        ).strip()
        self.eye_to_hand_affine: Optional[EyeToHandAffine] = None
        if self.eye_to_hand_enabled:
            self.eye_to_hand_affine = EyeToHandAffine.from_parameters(
                self.get_parameter("eye_to_hand_matrix").value,
                self.get_parameter("eye_to_hand_translation_mm").value,
            )
        if not all(
            (
                self.input_topic,
                self.output_topic,
                self.status_topic,
                self.output_frame_id,
            )
        ):
            raise ValueError(
                "Vision topic names and output_frame_id cannot be empty"
            )
        if min(
            self.expected_schema_version,
            self.maximum_detections,
            self.maximum_payload_bytes,
        ) <= 0:
            raise ValueError("Vision schema and input limits must be positive")
        if self.eye_to_hand_enabled and not (
            self.eye_to_hand_frame_id and self.eye_to_hand_calibration
        ):
            raise ValueError(
                "Enabled eye-to-hand calibration requires frame and name"
            )

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(
            VisionDetectionArray,
            self.output_topic,
            output_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            self.status_topic,
            status_qos,
        )
        self._subscription = self.create_subscription(
            String,
            self.input_topic,
            self._detection_callback,
            input_qos,
        )
        self._last_status = ""
        self._publish_status(
            f"WAITING: no detection batch received from {self.input_topic}"
        )
        self.get_logger().info(
            f"Vision bridge ready: {self.input_topic} -> {self.output_topic}"
        )

    def _publish_status(self, text: str) -> None:
        if text == self._last_status:
            return
        self._last_status = text
        self._status_publisher.publish(String(data=text))

    def _to_ros_message(
        self,
        batch: ModelDetectionBatch,
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp.sec = batch.stamp_sec
        message.header.stamp.nanosec = batch.stamp_nanosec
        message.header.frame_id = self.output_frame_id
        message.schema_version = batch.schema_version
        message.frame_sequence = batch.frame_sequence
        message.source = batch.source
        message.coordinate_space = batch.coordinate_space
        message.finish_edge = batch.finish_edge
        message.conveyor_width_m = batch.conveyor_width_m
        message.conveyor_length_m = batch.conveyor_length_m
        if self.eye_to_hand_enabled:
            message.eye_to_hand_frame_id = self.eye_to_hand_frame_id
            message.eye_to_hand_calibration = self.eye_to_hand_calibration
        for item in batch.detections:
            detection = VisionDetection()
            detection.track_id = item.track_id
            detection.identity_source = item.identity_source
            detection.label = item.label
            detection.confidence = item.confidence
            detection.is_danger = item.is_danger
            detection.geometry_valid = item.geometry_valid
            detection.projected_center_valid = item.projected_center_valid
            detection.projected_u_px = item.projected_u_px
            detection.projected_v_px = item.projected_v_px
            if self.eye_to_hand_enabled and item.geometry_valid:
                assert self.eye_to_hand_affine is not None
                eye_x_mm, eye_y_mm = self.eye_to_hand_affine.transform(
                    center_x_m=item.center_x_m,
                    center_s_m=item.center_s_m,
                    conveyor_length_m=batch.conveyor_length_m,
                    finish_edge=batch.finish_edge,
                )
                detection.eye_to_hand_valid = True
                detection.eye_to_hand_x_mm = eye_x_mm
                detection.eye_to_hand_y_mm = eye_y_mm
            detection.center_x_m = item.center_x_m
            detection.center_s_m = item.center_s_m
            detection.width_m = item.width_m
            detection.length_m = item.length_m
            detection.size_profile = item.size_profile
            message.detections.append(detection)
        return message

    def _detection_callback(self, message: String) -> None:
        try:
            batch = parse_model_detection_batch(
                message.data,
                expected_schema_version=self.expected_schema_version,
                maximum_detections=self.maximum_detections,
                maximum_payload_bytes=self.maximum_payload_bytes,
            )
        except DetectionSchemaError as exc:
            status = f"ERROR: rejected detection batch: {exc}"
            if status != self._last_status:
                self.get_logger().error(status)
            self._publish_status(status)
            return

        self._publisher.publish(self._to_ros_message(batch))
        danger_counts = label_counts(
            tuple(item for item in batch.detections if item.is_danger)
        )
        normal_counts = label_counts(
            tuple(item for item in batch.detections if not item.is_danger)
        )
        danger_text = ", ".join(
            f"{label}:{count}" for label, count in danger_counts
        ) or "none"
        normal_text = ", ".join(
            f"{label}:{count}" for label, count in normal_counts
        ) or "none"
        self._publish_status(
            f"OK: {len(batch.detections)} objects; "
            f"dangerous={danger_text}; normal={normal_text}; "
            f"eye_to_hand={self.eye_to_hand_calibration}"
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the external-detection bridge."""
    rclpy.init(args=args)
    node: Optional[FanucpyVisionBridge] = None
    try:
        node = FanucpyVisionBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
