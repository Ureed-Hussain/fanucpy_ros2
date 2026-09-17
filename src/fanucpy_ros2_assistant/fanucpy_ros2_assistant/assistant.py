#!/usr/bin/env python3
# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Unified conversational FANUC assistant with guarded visual XY alignment."""

import argparse
import math
import os
from pathlib import Path
import shlex
import sys
import time
from typing import Optional, Sequence, Tuple

import rclpy
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from rclpy.utilities import remove_ros_args

from fanucpy_ros2_interfaces.msg import DriverStatus, VisionDetectionArray
from fanucpy_ros2_task_planner.conversation import (
    CartesianSnapshot,
    format_absolute_cartesian_target,
    format_snapshot,
)
from fanucpy_ros2_task_planner.ollama_client import (
    ModelDecision,
    OllamaPlannerError,
)
from fanucpy_ros2_task_planner.ollama_prompt import (
    FanucpyOllamaPrompt,
    _execute_decision,
    _print_decision,
    print_status,
    process_prompt,
)
from fanucpy_ros2_task_planner.prompt_parser import PromptParseError
from fanucpy_ros2_task_planner.task_plans import (
    CartesianTargetPlan,
    TpProgramPlan,
    validate_tp_program_plan,
)

from .example_programs import (
    ExampleCatalog,
    ExampleError,
    check_conflicts,
    parse_example_request,
    run_foreground,
)
from .named_tasks import (
    _task_words,
    TaskCatalog,
    TaskFileError,
    TaskRequest,
    parse_task_request,
    validate_task,
)
from .package_help import PackageConversation
from .vision_motion import (
    VisionCartesianTarget,
    VisionMotionError,
    VisionMotionSelection,
    build_vision_cartesian_target,
    looks_like_vision_motion,
    parse_vision_motion,
    requested_cartesian_velocity,
    requests_maximum_velocity,
    select_visible_track,
    target_xy_shift_mm,
)
from .workflows import (
    DROP_TARGET_INTENT,
    HOME_PROGRAM_INTENT,
    PICK_INTENT,
    PickMotionPlan,
    build_named_cartesian_target,
    build_pick_motion_plan,
    is_workflow_question,
    pick_selection_prompt,
    special_intent,
    workflow_request_issue,
)


class FanucpyAssistant(FanucpyOllamaPrompt):
    """Add a separately gated vision-to-XY stage to the Ollama task client."""

    def __init__(
        self,
        model_override: Optional[str] = None,
        url_override: Optional[str] = None,
    ) -> None:
        super().__init__(
            model_override=model_override,
            url_override=url_override,
            node_name="fanucpy_assistant",
        )
        self.declare_parameter("enable_vision_guided_motion", False)
        self.declare_parameter("default_vision_velocity_mm_s", 25)
        self.declare_parameter("max_vision_xy_travel_mm", 500.0)
        self.declare_parameter("max_vision_target_refresh_shift_mm", 5.0)
        self.declare_parameter("vision_refresh_timeout_sec", 2.0)
        self.declare_parameter("vision_xy_offset_mm", [0.0, 0.0])
        self.declare_parameter("enable_pick_motion_sequence", False)
        self.declare_parameter("pick_approach_z_mm", -190.0)
        self.declare_parameter("pick_descend_z_mm", -238.0)
        self.declare_parameter("pick_retract_z_mm", -190.0)
        self.declare_parameter("max_pick_approach_z_change_mm", 100.0)
        self.declare_parameter("max_pick_vertical_stage_mm", 50.0)
        self.declare_parameter("pick_pose_tolerance_mm", 2.0)
        self.declare_parameter("pick_orientation_tolerance_deg", 1.0)
        self.declare_parameter("enable_drop_target_motion", False)
        self.declare_parameter(
            "drop_target_mm_deg",
            [237.0, 720.0, -167.0, -179.0, 60.0, -175.0],
        )
        self.declare_parameter("drop_velocity_mm_s", 25)
        self.declare_parameter("enable_home_program_call", False)
        self.declare_parameter("home_program_name", "HOME_P")

        self.enable_vision_guided_motion = bool(
            self.get_parameter("enable_vision_guided_motion").value
        )
        self.default_vision_velocity_mm_s = int(
            self.get_parameter("default_vision_velocity_mm_s").value
        )
        self.max_vision_xy_travel_mm = float(
            self.get_parameter("max_vision_xy_travel_mm").value
        )
        self.max_vision_target_refresh_shift_mm = float(
            self.get_parameter("max_vision_target_refresh_shift_mm").value
        )
        self.vision_refresh_timeout_sec = float(
            self.get_parameter("vision_refresh_timeout_sec").value
        )
        self.vision_xy_offset_mm = tuple(
            float(value)
            for value in self.get_parameter("vision_xy_offset_mm").value
        )
        self.enable_pick_motion_sequence = bool(
            self.get_parameter("enable_pick_motion_sequence").value
        )
        self.pick_approach_z_mm = float(
            self.get_parameter("pick_approach_z_mm").value
        )
        self.pick_descend_z_mm = float(
            self.get_parameter("pick_descend_z_mm").value
        )
        self.pick_retract_z_mm = float(
            self.get_parameter("pick_retract_z_mm").value
        )
        self.max_pick_approach_z_change_mm = float(
            self.get_parameter("max_pick_approach_z_change_mm").value
        )
        self.max_pick_vertical_stage_mm = float(
            self.get_parameter("max_pick_vertical_stage_mm").value
        )
        self.pick_pose_tolerance_mm = float(
            self.get_parameter("pick_pose_tolerance_mm").value
        )
        self.pick_orientation_tolerance_deg = float(
            self.get_parameter("pick_orientation_tolerance_deg").value
        )
        self.enable_drop_target_motion = bool(
            self.get_parameter("enable_drop_target_motion").value
        )
        self.drop_target_mm_deg = tuple(
            float(value)
            for value in self.get_parameter("drop_target_mm_deg").value
        )
        self.drop_velocity_mm_s = int(
            self.get_parameter("drop_velocity_mm_s").value
        )
        self.enable_home_program_call = bool(
            self.get_parameter("enable_home_program_call").value
        )
        self.home_program_name = str(
            self.get_parameter("home_program_name").value
        ).strip().upper()
        if self.default_vision_velocity_mm_s <= 0:
            raise ValueError("default_vision_velocity_mm_s must be positive")
        if not math.isfinite(self.max_vision_xy_travel_mm) or (
            self.max_vision_xy_travel_mm <= 0.0
        ):
            raise ValueError("max_vision_xy_travel_mm must be positive")
        if not math.isfinite(self.max_vision_target_refresh_shift_mm) or (
            self.max_vision_target_refresh_shift_mm < 0.0
        ):
            raise ValueError(
                "max_vision_target_refresh_shift_mm must not be negative"
            )
        if self.vision_refresh_timeout_sec <= 0.0:
            raise ValueError("vision_refresh_timeout_sec must be positive")
        if len(self.vision_xy_offset_mm) != 2 or not all(
            math.isfinite(value) for value in self.vision_xy_offset_mm
        ):
            raise ValueError(
                "vision_xy_offset_mm must contain two finite values"
            )
        if not all(
            math.isfinite(value)
            for value in (
                self.pick_approach_z_mm,
                self.pick_descend_z_mm,
                self.pick_retract_z_mm,
            )
        ):
            raise ValueError("Pick Z targets must be finite")
        if self.pick_descend_z_mm >= self.pick_approach_z_mm:
            raise ValueError("pick_descend_z_mm must be below approach Z")
        if self.pick_retract_z_mm < self.pick_descend_z_mm:
            raise ValueError("pick_retract_z_mm must be above descent Z")
        if min(
            self.max_pick_approach_z_change_mm,
            self.max_pick_vertical_stage_mm,
            self.pick_pose_tolerance_mm,
            self.pick_orientation_tolerance_deg,
        ) <= 0.0:
            raise ValueError("Pick limits and tolerances must be positive")
        if len(self.drop_target_mm_deg) != 6 or not all(
            math.isfinite(value) for value in self.drop_target_mm_deg
        ):
            raise ValueError(
                "drop_target_mm_deg must contain six finite values"
            )
        if self.drop_velocity_mm_s <= 0:
            raise ValueError("drop_velocity_mm_s must be positive")
        validate_tp_program_plan(TpProgramPlan(self.home_program_name))
        self.package_conversation = PackageConversation()
        self.declare_parameter("task_directory", "")
        # An empty-string sentinel gives ROS an explicitly string-typed array.
        self.declare_parameter("allowed_tasks", [""])
        directory = str(self.get_parameter("task_directory").value).strip()
        using_builtin_tasks = not directory
        if not directory:
            directory = str(Path(get_package_share_directory(
                "fanucpy_ros2_assistant"
            )) / "tasks")
        try:
            self.task_catalog = TaskCatalog(
                directory,
                allow_external_symlinks=using_builtin_tasks,
            )
        except (OSError, ValueError) as exc:
            raise ValueError(f"Cannot load task_directory: {exc}") from exc
        self.allowed_tasks = frozenset(
            name for name in self.get_parameter("allowed_tasks").value if name
        )
        unknown = self.allowed_tasks.difference(self.task_catalog.names)
        if unknown:
            raise ValueError(f"allowed_tasks contains unknown tasks: {unknown}")
        self.declare_parameter("example_registry_file", "")
        self.declare_parameter("allowed_examples", [""])
        registry = str(self.get_parameter("example_registry_file").value).strip()
        if not registry:
            registry = str(Path(get_package_share_directory(
                "fanucpy_ros2_assistant"
            )) / "config" / "examples.json")
        try:
            self.example_catalog = ExampleCatalog(registry)
        except (OSError, ValueError) as exc:
            raise ValueError(f"Cannot load example registry: {exc}") from exc
        self.allowed_examples = frozenset(
            name for name in self.get_parameter("allowed_examples").value
            if name
        )
        unknown = self.allowed_examples.difference(self.example_catalog.names)
        if unknown:
            raise ValueError(f"allowed_examples has unknown entries: {unknown}")
        if any(
            self.example_catalog.resolve(_task_words(name))
            for name in self.task_catalog.names
        ):
            raise ValueError("Task and example names must not overlap")

    def maximum_cartesian_velocity(self) -> int:
        """Return the active driver ceiling or the local fallback."""
        status = self.latest_status
        if status is not None and status.max_cartesian_velocity_mm_s > 0:
            return int(status.max_cartesian_velocity_mm_s)
        return self.parser.max_cartesian_velocity_mm_s

    def workflow_velocity_limit(self, prompt: str) -> int:
        """Resolve explicit maximum speed only with connected live feedback."""
        if requests_maximum_velocity(prompt):
            state = self.wait_for_cartesian_state(
                timeout_sec=self.robot_context_timeout_sec,
                require_fresh=True,
                received_after=time.monotonic(),
            )
            status = self.latest_status
            if (
                state is None
                or status is None
                or status.state != DriverStatus.CONNECTED
                or status.max_cartesian_velocity_mm_s <= 0
            ):
                raise VisionMotionError(
                    "I need fresh robot feedback and a connected driver's "
                    "speed limit to interpret 'max speed'. Please reconnect "
                    "the driver or specify a speed in mm/s."
                )
        return self.maximum_cartesian_velocity()

    def validate_vision_frame(
        self,
        message: VisionDetectionArray,
        expected_calibration: str = "",
    ) -> None:
        """Require a named calibration in the configured command frame."""
        frame_id = message.eye_to_hand_frame_id.strip()
        calibration = message.eye_to_hand_calibration.strip()
        if not frame_id or frame_id != self.frame_id:
            raise VisionMotionError(
                "Vision eye-to-hand frame does not match the command frame: "
                f"{frame_id or 'unavailable'} != {self.frame_id}"
            )
        if not calibration:
            raise VisionMotionError(
                "The vision batch does not identify an eye-to-hand calibration"
            )
        if expected_calibration and calibration != expected_calibration:
            raise VisionMotionError(
                "Eye-to-hand calibration changed after preview: "
                f"{expected_calibration} -> {calibration}"
            )


def _selection_name(selection: VisionMotionSelection) -> str:
    """Return the human alias assigned in the current frame."""
    alias = selection.alias
    observation = alias.observation
    if alias.family in {"battery", "bottle"} or (
        alias.family != observation.label
    ):
        name = alias.family.replace("-", " ")
        ordinal = alias.family_ordinal
    else:
        name = observation.label.replace("-", " ")
        ordinal = alias.exact_ordinal
    return f"{name.capitalize()} {ordinal}"


def _build_target(
    node: FanucpyAssistant,
    selection: VisionMotionSelection,
    snapshot: CartesianSnapshot,
) -> VisionCartesianTarget:
    """Construct and locally validate one selected XY alignment target."""
    return build_vision_cartesian_target(
        selection=selection,
        current_values=snapshot.values,
        xy_offset_mm=node.vision_xy_offset_mm,
        max_xy_travel_mm=node.max_vision_xy_travel_mm,
        max_velocity_mm_s=node.maximum_cartesian_velocity(),
    )


def _print_target_preview(
    node: FanucpyAssistant,
    message: VisionDetectionArray,
    snapshot: CartesianSnapshot,
    target: VisionCartesianTarget,
) -> None:
    """Print the measured source and exact resolved controller target."""
    observation = target.selection.alias.observation
    name = _selection_name(target.selection)
    category = "dangerous" if observation.is_danger else "normal"
    print(
        f"\nAssistant: I found {name} ({observation.label}; {category}; "
        f"track ID {observation.track_id}). I interpret your request as XY "
        "alignment above that object while keeping the current Z, W, P, and R."
    )
    print(
        "Vision source: "
        f"frame sequence {message.frame_sequence}; "
        f"frame={message.eye_to_hand_frame_id}; "
        f"calibration={message.eye_to_hand_calibration}"
    )
    print(
        "Measured object XY: "
        f"[{observation.eye_to_hand_x_mm:.3f}, "
        f"{observation.eye_to_hand_y_mm:.3f}] mm"
    )
    print(
        "Configured TCP XY offset: "
        f"[{node.vision_xy_offset_mm[0]:+.3f}, "
        f"{node.vision_xy_offset_mm[1]:+.3f}] mm"
    )
    print(format_snapshot(snapshot))
    print(
        format_absolute_cartesian_target(
            node.frame_id,
            target.resolved_plan,
        )
    )
    print(f"Requested speed: {target.resolved_plan.velocity_mm_s} mm/s")
    print(f"Planned horizontal XY travel: {target.xy_travel_mm:.3f} mm")
    print(
        "Z and W/P/R are preserved from fresh robot feedback. This command "
        "does not descend, grasp, track a moving conveyor, or check "
        "collisions."
    )


def _confirm_vision_target(track_id: int) -> bool:
    """Require an object-specific confirmation in an interactive terminal."""
    if not sys.stdin.isatty():
        print("Execution blocked: an interactive terminal is required")
        return False
    token = f"MOVE TO TRACK {track_id}"
    print(
        "Confirm that the conveyor and selected object are stationary, the "
        "camera calibration and active FANUC frame are correct, and the full "
        "horizontal path is clear. Keep pendant HOLD and emergency stop ready."
    )
    return input(f"Type {token} to send this XY-alignment target: ") == token


def _refresh_target_after_confirmation(
    node: FanucpyAssistant,
    preview_message: VisionDetectionArray,
    preview: VisionCartesianTarget,
    confirmed_at: float,
) -> Tuple[VisionDetectionArray, CartesianSnapshot, VisionCartesianTarget]:
    """Reacquire the same track and robot state immediately before sending."""
    message = node.wait_for_vision(
        timeout_sec=node.vision_refresh_timeout_sec,
        received_after=confirmed_at,
    )
    if message is None:
        raise VisionMotionError(
            "No new fresh vision frame arrived after confirmation"
        )
    node.validate_vision_frame(
        message,
        expected_calibration=preview_message.eye_to_hand_calibration,
    )
    state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
        received_after=confirmed_at,
    )
    if state is None:
        raise VisionMotionError(
            "No new fresh Cartesian state arrived after confirmation"
        )
    snapshot = node.cartesian_snapshot(state)
    original = preview.selection.alias.observation
    refreshed_selection = select_visible_track(
        observations=node.vision_observations(message),
        track_id=original.track_id,
        expected_label=original.label,
        velocity_mm_s=preview.selection.velocity_mm_s,
        expected_identity_source=original.identity_source,
    )
    refreshed = _build_target(node, refreshed_selection, snapshot)
    shift = target_xy_shift_mm(preview, refreshed)
    if shift > node.max_vision_target_refresh_shift_mm:
        raise VisionMotionError(
            f"The selected target shifted {shift:.3f} mm after preview, above "
            f"the configured {node.max_vision_target_refresh_shift_mm:.3f} mm "
            "limit; issue a new command after the scene is stationary"
        )
    return message, snapshot, refreshed


def _execute_vision_target(
    node: FanucpyAssistant,
    preview_message: VisionDetectionArray,
    preview: VisionCartesianTarget,
) -> bool:
    """Apply every gate, refresh the target, and send one absolute action."""
    if not node.enable_vision_guided_motion:
        print(
            "EXECUTION BLOCKED: enable_vision_guided_motion is false. The "
            "vision target was previewed but not sent."
        )
        return False
    try:
        node.check_absolute_cartesian_execution_ready(preview.resolved_plan)
    except (PromptParseError, RuntimeError) as exc:
        print(f"EXECUTION BLOCKED: {exc}")
        return False
    track_id = preview.selection.alias.observation.track_id
    if not _confirm_vision_target(track_id):
        print("Vision-guided motion cancelled; nothing was sent.")
        return False
    confirmed_at = time.monotonic()
    try:
        message, snapshot, refreshed = _refresh_target_after_confirmation(
            node,
            preview_message,
            preview,
            confirmed_at,
        )
        node.check_absolute_cartesian_execution_ready(refreshed.resolved_plan)
    except (PromptParseError, RuntimeError, VisionMotionError) as exc:
        print(f"EXECUTION BLOCKED AFTER CONFIRMATION: {exc}")
        return False
    print("\nFinal refreshed target immediately before execution")
    _print_target_preview(node, message, snapshot, refreshed)
    if not node.execute_absolute_cartesian(refreshed.resolved_plan):
        return False
    completed_at = time.monotonic()
    result = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
        received_after=completed_at,
    )
    if result is not None:
        print("Actual Cartesian state after vision-guided action")
        print(format_snapshot(node.cartesian_snapshot(result)))
    return True


def _confirmation(token: str, description: str) -> bool:
    """Require one workflow-specific confirmation token."""
    if not sys.stdin.isatty():
        print("Execution blocked: an interactive terminal is required")
        return False
    return input(f"Type {token} to {description}: ") == token


def _workflow_disabled(parameter: str) -> None:
    """Explain an execution gate with the exact startup override needed."""
    print(
        f"Assistant: The plan is ready, but {parameter} is disabled. "
        "Nothing was sent. Restart the assistant with --execute and "
        f"-p {parameter}:=true after --ros-args, keeping your other options."
    )


def _pick_plan(
    node: FanucpyAssistant,
    selection: VisionMotionSelection,
    snapshot: CartesianSnapshot,
) -> PickMotionPlan:
    """Construct the configured three-stage pick-motion plan."""
    return build_pick_motion_plan(
        selection=selection,
        current_values=snapshot.values,
        xy_offset_mm=node.vision_xy_offset_mm,
        approach_z_mm=node.pick_approach_z_mm,
        descend_z_mm=node.pick_descend_z_mm,
        retract_z_mm=node.pick_retract_z_mm,
        max_xy_travel_mm=node.max_vision_xy_travel_mm,
        max_approach_z_change_mm=node.max_pick_approach_z_change_mm,
        max_vertical_stage_mm=node.max_pick_vertical_stage_mm,
        max_velocity_mm_s=node.maximum_cartesian_velocity(),
    )


def _print_pick_preview(
    node: FanucpyAssistant,
    message: VisionDetectionArray,
    snapshot: CartesianSnapshot,
    plan: PickMotionPlan,
) -> None:
    """Show every exact stage of the configured pick-motion sequence."""
    observation = plan.selection.alias.observation
    name = _selection_name(plan.selection)
    category = "dangerous" if observation.is_danger else "normal"
    print(
        f"\nAssistant: I found {name} ({observation.label}; {category}; "
        f"track ID {observation.track_id}). I'll move above it, lower to "
        f"Z={plan.descend.z_mm:g} mm, and return to "
        f"Z={plan.retract.z_mm:g} mm at {plan.approach.velocity_mm_s} mm/s. "
        "This sequence moves the robot only; the gripper is not operated."
    )
    print(
        "Vision source: "
        f"frame sequence {message.frame_sequence}; "
        f"frame={message.eye_to_hand_frame_id}; "
        f"calibration={message.eye_to_hand_calibration}"
    )
    print(format_snapshot(snapshot))
    print(
        "Stage 1 - approach object XY:\n"
        + format_absolute_cartesian_target(node.frame_id, plan.approach)
    )
    print(
        "Stage 2 - vertical descent only:\n"
        + format_absolute_cartesian_target(node.frame_id, plan.descend)
    )
    print(
        "Stage 3 - vertical retraction only:\n"
        + format_absolute_cartesian_target(node.frame_id, plan.retract)
    )
    print(f"Planned approach XY travel: {plan.xy_travel_mm:.3f} mm")
    print(
        "Requested speed for all three stages: "
        f"{plan.approach.velocity_mm_s} mm/s"
    )
    print(
        "The conveyor and object must remain stationary. This sequence does "
        "not calculate collisions, object height, tool clearance, grasping, "
        "or moving-conveyor interception."
    )


def _angle_error_deg(actual: float, target: float) -> float:
    """Return the shortest absolute angular error in degrees."""
    return abs((actual - target + 180.0) % 360.0 - 180.0)


def _execute_cartesian_stage(
    node: FanucpyAssistant,
    stage_name: str,
    plan: CartesianTargetPlan,
) -> bool:
    """Execute and verify one stage before a sequence may continue."""
    try:
        node.check_absolute_cartesian_execution_ready(plan)
    except (PromptParseError, RuntimeError) as exc:
        print(f"{stage_name} BLOCKED: {exc}")
        return False
    print(f"\nExecuting {stage_name}...")
    if not node.execute_absolute_cartesian(plan):
        print(
            f"{stage_name} failed or its final state is uncertain. No later "
            "workflow stage will be sent automatically."
        )
        return False
    completed_at = time.monotonic()
    state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
        received_after=completed_at,
    )
    if state is None:
        print(
            f"{stage_name} completed at the action layer, but no fresh "
            "Cartesian feedback arrived. No later stage will be sent."
        )
        return False
    snapshot = node.cartesian_snapshot(state)
    translation_errors = tuple(
        abs(actual - target)
        for actual, target in zip(snapshot.values[:3], plan.target[:3])
    )
    orientation_errors = tuple(
        _angle_error_deg(actual, target)
        for actual, target in zip(snapshot.values[3:], plan.target[3:])
    )
    if max(translation_errors) > node.pick_pose_tolerance_mm or (
        max(orientation_errors) > node.pick_orientation_tolerance_deg
    ):
        print(
            f"{stage_name} feedback is outside tolerance; actual state:\n"
            + format_snapshot(snapshot)
        )
        print("No later workflow stage will be sent automatically.")
        return False
    print(f"{stage_name} verified from fresh Cartesian feedback.")
    return True


def _process_pick_prompt(
    node: FanucpyAssistant,
    prompt: str,
    execution_enabled: bool,
) -> int:
    """Plan or execute a guarded object-centered down/up motion sequence."""
    message = node.wait_for_vision(timeout_sec=0.25)
    if message is None:
        print(
            "\nAssistant: I cannot pick because fresh vision is "
            "unavailable."
        )
        return 2
    try:
        node.validate_vision_frame(message)
        selection = parse_vision_motion(
            prompt=pick_selection_prompt(prompt),
            observations=node.vision_observations(message),
            default_velocity_mm_s=node.default_vision_velocity_mm_s,
            max_velocity_mm_s=node.workflow_velocity_limit(prompt),
        )
        if selection is None:
            raise VisionMotionError(
                "Name a visible object, object number, or track ID to pick"
            )
        state = node.wait_for_cartesian_state(
            timeout_sec=node.robot_context_timeout_sec,
            require_fresh=True,
        )
        if state is None:
            raise VisionMotionError("Fresh Cartesian feedback is unavailable")
        snapshot = node.cartesian_snapshot(state)
        latest_message = node.wait_for_vision(timeout_sec=0.0) or message
        node.validate_vision_frame(latest_message)
        selected = selection.alias.observation
        selection = select_visible_track(
            observations=node.vision_observations(latest_message),
            track_id=selected.track_id,
            expected_label=selected.label,
            velocity_mm_s=selection.velocity_mm_s,
            expected_identity_source=selected.identity_source,
        )
        alignment = _build_target(node, selection, snapshot)
        plan = _pick_plan(node, selection, snapshot)
    except VisionMotionError as exc:
        print(f"\nAssistant: I cannot create that pick sequence: {exc}")
        return 2
    _print_pick_preview(node, latest_message, snapshot, plan)
    if not execution_enabled:
        print("DRY RUN: the three pick-motion stages were not sent.")
        return 0
    if not node.enable_vision_guided_motion:
        _workflow_disabled("enable_vision_guided_motion")
        return 1
    if not node.enable_pick_motion_sequence:
        _workflow_disabled("enable_pick_motion_sequence")
        return 1
    for stage in (plan.approach, plan.descend, plan.retract):
        try:
            node.check_absolute_cartesian_execution_ready(stage)
        except (PromptParseError, RuntimeError) as exc:
            print(f"EXECUTION BLOCKED: {exc}")
            return 1
    track_id = selection.alias.observation.track_id
    print(
        "Confirm the complete approach, descent, and retraction path is "
        "clear; the object and conveyor are stationary; and pendant HOLD "
        "and emergency stop are available."
    )
    if not _confirmation(
        f"PICK TRACK {track_id}",
        "send all three motion-only pick stages",
    ):
        print("Pick-motion sequence cancelled; nothing was sent.")
        return 1
    confirmed_at = time.monotonic()
    try:
        refreshed_message, refreshed_snapshot, refreshed_alignment = (
            _refresh_target_after_confirmation(
                node,
                latest_message,
                alignment,
                confirmed_at,
            )
        )
        refreshed_plan = _pick_plan(
            node,
            refreshed_alignment.selection,
            refreshed_snapshot,
        )
    except VisionMotionError as exc:
        print(f"EXECUTION BLOCKED AFTER CONFIRMATION: {exc}")
        return 1
    print("\nFinal refreshed pick-motion plan immediately before execution")
    _print_pick_preview(
        node,
        refreshed_message,
        refreshed_snapshot,
        refreshed_plan,
    )
    stages = (
        ("pick approach", refreshed_plan.approach),
        ("pick descent", refreshed_plan.descend),
        ("pick retraction", refreshed_plan.retract),
    )
    for stage_name, stage in stages:
        if not _execute_cartesian_stage(node, stage_name, stage):
            return 1
    print(
        "Assistant: The approach, descent, and retraction are complete. "
        "The gripper was not operated. You can say 'drop it at 200 mm/s' "
        "to move to the saved drop pose, or 'go home' to call the home "
        "program."
    )
    return 0


def _drop_plan(node: FanucpyAssistant, prompt: str) -> CartesianTargetPlan:
    """Use the saved drop pose and the speed requested in this message."""
    maximum = node.workflow_velocity_limit(prompt)
    velocity = requested_cartesian_velocity(
        prompt, node.drop_velocity_mm_s, maximum
    )
    return build_named_cartesian_target(
        node.drop_target_mm_deg,
        velocity,
        maximum,
    )


def _process_drop_prompt(
    node: FanucpyAssistant,
    prompt: str,
    execution_enabled: bool,
) -> int:
    """Preview or execute the configured named drop-position target."""
    try:
        plan = _drop_plan(node, prompt)
    except VisionMotionError as exc:
        print(f"\nAssistant: I cannot prepare that drop movement: {exc}")
        return 2
    print(
        "\nAssistant: I'll move to your saved drop position at "
        f"{plan.velocity_mm_s} mm/s. This moves the robot only; "
        "it does not release an object or operate the gripper."
    )
    if requests_maximum_velocity(prompt):
        print(
            "'Max speed' resolved to the connected driver's configured "
            f"Cartesian ceiling: {plan.velocity_mm_s} mm/s."
        )
    print(format_absolute_cartesian_target(node.frame_id, plan))
    if not execution_enabled:
        print("DRY RUN: the drop-position target was not sent.")
        return 0
    if not node.enable_drop_target_motion:
        _workflow_disabled("enable_drop_target_motion")
        return 1
    try:
        node.check_absolute_cartesian_execution_ready(plan)
    except (PromptParseError, RuntimeError) as exc:
        print(f"EXECUTION BLOCKED: {exc}")
        return 1
    print(
        "Confirm the configured drop pose, active frame, tooling, and entire "
        "path are safe. Keep pendant HOLD and emergency stop available."
    )
    if not _confirmation("MOVE TO DROP", "send the configured drop target"):
        print("Drop-position movement cancelled; nothing was sent.")
        return 1
    if not _execute_cartesian_stage(node, "drop target", plan):
        return 1
    print(
        "Assistant: The robot reached the saved drop position. "
        "The gripper was not operated. You can say 'go home' next."
    )
    return 0


def _process_home_prompt(
    node: FanucpyAssistant,
    execution_enabled: bool,
) -> int:
    """Preview or call the configured allowlisted home TP program."""
    plan = validate_tp_program_plan(TpProgramPlan(node.home_program_name))
    print(
        f"\nAssistant: I'll call your configured home program "
        f"{plan.program_name}. Its motion and speed are defined on the "
        "controller."
    )
    if not execution_enabled:
        print(f"DRY RUN: TP program {plan.program_name} was not called.")
        return 0
    if not node.enable_home_program_call:
        _workflow_disabled("enable_home_program_call")
        return 1
    try:
        checked = node.check_program_execution_ready(plan)
    except (PromptParseError, RuntimeError) as exc:
        print(f"EXECUTION BLOCKED: {exc}")
        return 1
    print(
        "Confirm that the reviewed TP program is safe from the robot's "
        "current state and pendant HOLD and emergency stop are available."
    )
    if not _confirmation(
        f"RUN {checked.program_name}",
        "call this allowlisted TP program once",
    ):
        print("Home program cancelled; nothing was sent.")
        return 1
    try:
        checked = node.check_program_execution_ready(checked)
    except (PromptParseError, RuntimeError) as exc:
        print(f"EXECUTION BLOCKED AFTER CONFIRMATION: {exc}")
        return 1
    return 0 if node.execute_tp_program(checked) else 1


def _preflight_example(node, program):
    """Inspect installed files and live graph without starting any program."""
    try:
        if program.kind == "run":
            target = Path(get_package_prefix(program.package)) / (
                "lib"
            ) / program.package / program.target
            if not target.is_file() or not os.access(target, os.X_OK):
                raise ExampleError(f"Installed executable unavailable: {target}")
        else:
            target = Path(get_package_share_directory(program.package)) / (
                "launch"
            ) / program.target
            if not target.is_file():
                raise ExampleError(
                    f"Installed launch file unavailable: {target}. "
                    "Build/source the updated package first."
                )
    except PackageNotFoundError as exc:
        raise ExampleError(
            f"Package {program.package} is unavailable; build and source it"
        ) from exc
    # Allow local discovery to settle. This is a best-effort graph check,
    # not an inter-process lock or proof that the workcell is idle/safe.
    deadline = time.monotonic() + 0.5
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if program.requires_driver:
        state = node.wait_for_cartesian_state(
            timeout_sec=node.robot_context_timeout_sec,
            require_fresh=True,
            received_after=time.monotonic(),
        )
        status = node.latest_status
        if (
            state is None or status is None
            or status.state != DriverStatus.CONNECTED
        ):
            raise ExampleError(
                "A connected existing driver and fresh Cartesian feedback "
                "are required. Start your support bringup separately."
            )
        node_names = [name for name, _ in node.get_node_names_and_namespaces()]
        if node_names.count("fanucpy_driver") != 1:
            raise ExampleError("Expected exactly one existing fanucpy_driver node")
    else:
        node_names = [name for name, _ in node.get_node_names_and_namespaces()]
    check_conflicts(program, node_names)
    if program.requires_joint_states:
        publishers = node.get_publishers_info_by_topic("/joint_states")
        if len(publishers) != 1 or publishers[0].node_name != "fanucpy_driver":
            raise ExampleError(
                "MoveIt requires exactly one /joint_states publisher from "
                "fanucpy_driver; stop conflicting mock/joint-state publishers"
            )
        if node.wait_for_joint_state(
            timeout_sec=node.robot_context_timeout_sec,
            require_fresh=True,
            received_after=time.monotonic(),
        ) is None:
            raise ExampleError("Fresh joint feedback is required for MoveIt")


def _process_example(node, request, execution_enabled):
    """Hand the terminal to a reviewed program after explicit permission."""
    catalog = node.example_catalog
    if request.operation == "list":
        print("\nAssistant: Available ROS example programs:")
        for name in catalog.names:
            program = catalog.get(name)
            permission = (
                "allowlisted" if name in node.allowed_examples else "preview only"
            )
            print(f"  {name}: {program.description} [{permission}]")
        print("Use 'run teleop example' or 'preview moveit2 example'.")
        print("For saved numeric robot tasks, use /tasks instead.")
        return 0
    program = catalog.get(request.name)
    print(f"\nAssistant: {program.description}")
    print(f"Command: {shlex.join(program.argv)}")
    print(f"Registry: {catalog.source}\nSHA-256: {catalog.sha256}")
    if request.operation == "preview" or not execution_enabled:
        print("DRY RUN: no process was started.")
        return 0
    if program.name not in node.allowed_examples:
        raise ExampleError(
            f"Example {program.name} is not in allowed_examples. Review its "
            "command before restarting with that exact name allowlisted."
        )
    _preflight_example(node, program)
    print(
        "The example will own this terminal until it exits. The assistant "
        "will not accept simultaneous robot prompts. Review the installed "
        "program and keep the workcell safeguarded. Examples use their OWN "
        "controls; assistant per-motion confirmations do not apply inside "
        "them. Existing driver gates remain unchanged."
    )
    if not _confirmation(f"LAUNCH {program.name}", "start this ROS example"):
        print("Example launch cancelled; no process was started.")
        return 1
    _preflight_example(node, program)
    result = run_foreground(program)
    print(
        f"\nAssistant: Example process exited with code {result}. "
        "Process exit/Ctrl+C is not confirmation that robot motion stopped. "
        "Use pendant HOLD/emergency stop when needed. No automatic restart."
    )
    return 0 if result == 0 else 1


def _process_named_task(node, request: TaskRequest, execution_enabled: bool):
    """Preview or execute one immutable task using existing guarded actions."""
    catalog = node.task_catalog
    if request.operation == "list":
        print("\nAssistant: These task files were loaded at startup:")
        for name in catalog.names:
            task = catalog.get(name)
            permission = (
                "read-only" if task.plan is None else
                "allowlisted; other execution gates still apply"
                if name in node.allowed_tasks else "preview only"
            )
            print(f"  {name}: {task.description} [{permission}]")
        print("Try 'preview TASK_NAME' or 'please run task TASK_NAME'.")
        return 0
    task = catalog.get(request.name)
    validate_task(task, node.parser, node.frame_id)
    print(f"\nTask file: {task.source}\nSHA-256: {task.sha256}")
    if task.plan is None:
        if request.operation == "preview":
            print(f"Assistant: {task.description} No status read requested.")
        else:
            _print_combined_status(node)
        return 0
    _, snapshot, joints = node.robot_context(require_fresh_state=True)
    decision = ModelDecision(
        message=f"Task {task.name}: {task.description}",
        needs_clarification=False,
        clarification_question="",
        plan=task.plan,
    )
    if not _print_decision(node, decision, snapshot, joints):
        return 2
    if request.operation == "preview" or not execution_enabled:
        print("DRY RUN: the saved task was not sent.")
        return 0
    if task.name not in node.allowed_tasks:
        print(
            "EXECUTION BLOCKED: this task is not in allowed_tasks. Review "
            "its file before restarting with that exact task allowlisted."
        )
        return 1
    # Reuse the same live checks and operator confirmation as direct prompts.
    # No subprocess, eval, dynamic import, shell, task chaining or retries.
    succeeded = _execute_decision(node, task.plan)
    print(
        "Assistant: The task action reported success."
        if succeeded else
        "Assistant: The task did not report success or was cancelled. "
        "I will not retry it. A timeout/interruption does not stop motion; "
        "check the controller before requesting another action."
    )
    return 0 if succeeded else 1


def process_assistant_prompt(
    node: FanucpyAssistant,
    prompt: str,
    execution_enabled: bool,
) -> int:
    """Route read-only help, saved tasks and existing guarded robot intents."""
    conversation = getattr(node, "package_conversation", None)
    if conversation is None:
        conversation = node.package_conversation = PackageConversation()
    answer = conversation.reply(prompt)
    if answer is not None:
        print(f"\nAssistant: {answer}")
        return 0
    catalog = getattr(node, "task_catalog", None)
    try:
        examples = getattr(node, "example_catalog", None)
        if examples is not None:
            request = parse_example_request(prompt, examples)
            if request is not None:
                return _process_example(node, request, execution_enabled)
        request = parse_task_request(prompt, catalog.names if catalog else ())
        if request is not None:
            if catalog is None:
                raise TaskFileError("Task catalog is unavailable")
            return _process_named_task(node, request, execution_enabled)
    except ExampleError as exc:
        print(f"\nAssistant: Example launch blocked or failed: {exc}")
        return 2
    except (TaskFileError, PromptParseError) as exc:
        print(f"\nAssistant: {exc} Nothing was sent.")
        return 2
    intent = special_intent(prompt)
    if intent is not None:
        issue = workflow_request_issue(prompt, intent)
        if issue is not None:
            print(f"\nAssistant: {issue}")
            return 2
        if is_workflow_question(prompt):
            print(
                "Assistant: Here is the workflow preview; "
                "nothing will be sent."
            )
            execution_enabled = False
    if intent == HOME_PROGRAM_INTENT:
        return _process_home_prompt(node, execution_enabled)
    if intent == DROP_TARGET_INTENT:
        return _process_drop_prompt(node, prompt, execution_enabled)
    if intent == PICK_INTENT:
        return _process_pick_prompt(node, prompt, execution_enabled)
    message = node.wait_for_vision(timeout_sec=0.25)
    observations = (
        node.vision_observations(message) if message is not None else ()
    )
    if not looks_like_vision_motion(prompt, observations):
        return process_prompt(node, prompt, execution_enabled)
    if message is None:
        print(
            "\nAssistant: I cannot plan object-directed motion because no "
            "fresh normalized vision frame is available."
        )
        return 2
    try:
        node.validate_vision_frame(message)
        selection = parse_vision_motion(
            prompt=prompt,
            observations=observations,
            default_velocity_mm_s=node.default_vision_velocity_mm_s,
            max_velocity_mm_s=node.workflow_velocity_limit(prompt),
        )
        if selection is None:
            return process_prompt(node, prompt, execution_enabled)
        state = node.wait_for_cartesian_state(
            timeout_sec=node.robot_context_timeout_sec,
            require_fresh=True,
        )
        if state is None:
            raise VisionMotionError(
                "Fresh Cartesian feedback is required because this stage "
                "preserves the current Z and W/P/R"
            )
        snapshot = node.cartesian_snapshot(state)
        latest_message = node.wait_for_vision(timeout_sec=0.0) or message
        node.validate_vision_frame(latest_message)
        selected = selection.alias.observation
        selection = select_visible_track(
            observations=node.vision_observations(latest_message),
            track_id=selected.track_id,
            expected_label=selected.label,
            velocity_mm_s=selection.velocity_mm_s,
            expected_identity_source=selected.identity_source,
        )
        target = _build_target(node, selection, snapshot)
    except VisionMotionError as exc:
        print(f"\nAssistant: I cannot create that visual target: {exc}")
        return 2
    _print_target_preview(node, latest_message, snapshot, target)
    if not execution_enabled:
        print(
            "DRY RUN: the visual XY target was validated but not sent. "
            "Restart with --execute to permit the separate execution gates."
        )
        return 0
    return 0 if _execute_vision_target(node, latest_message, target) else 1


def _print_help(execution_enabled: bool) -> None:
    """Show combined task-planner and visual-alignment examples."""
    mode = (
        "execution permitted with confirmation"
        if execution_enabled
        else "dry run"
    )
    print(
        "\nFANUC ROS 2 conversational assistant\n"
        "-------------------------------------\n"
        f"Mode: {mode}\n"
        "Vision examples:\n"
        "  what do you see?\n"
        "  tell me the coordinates of battery one\n"
        "  go to battery one\n"
        "  align with bottle 2 at 25 mm/s\n"
        "  move above track 12\n"
        "  please pick up battery one at 25 mm/s\n"
        "Named workflow examples:\n"
        "  drop it at 200 mm/s\n"
        "  take the robot to the drop position\n"
        "  go to drop position at max speed\n"
        "  please return the robot home\n"
        "Existing robot examples:\n"
        "  move left 5 mm at 20 mm/s\n"
        "  move X 300\n"
        "  move joints to J1 0 J2 0 J3 0 J4 0 J5 0 J6 0 at 5%\n"
        "  run TP program HOME_P\n"
        "Package help and saved tasks (no Ollama call required):\n"
        "  why is vision unavailable?\n"
        "  tell me more\n"
        "  /ask how do I build this package?\n"
        "  /tasks\n"
        "  preview move_left_demo\n"
        "  please run the move left example\n"
        "ROS program examples (separate allowlist and launch confirmation):\n"
        "  /examples\n"
        "  run teleop example\n"
        "  run moveit2 example\n"
        "Commands: /status, /tasks, /examples, /ask, /more, /clear, /help, /quit\n"
    )


def _print_combined_status(node: FanucpyAssistant) -> None:
    """Print existing robot status plus current visual-target metadata."""
    print_status(node)
    message = node.wait_for_vision(timeout_sec=0.25)
    if message is None:
        print("Vision: unavailable or stale")
        return
    print(
        "Vision: fresh; "
        f"frame_sequence={message.frame_sequence}; "
        f"objects={len(message.detections)}; "
        f"eye_frame={message.eye_to_hand_frame_id}; "
        f"calibration={message.eye_to_hand_calibration}"
    )
    print(
        "Vision-guided execution gate: "
        + ("enabled" if node.enable_vision_guided_motion else "disabled")
    )
    print(
        "Pick-motion sequence gate: "
        + ("enabled" if node.enable_pick_motion_sequence else "disabled")
    )
    print(
        "Drop-target motion gate: "
        + ("enabled" if node.enable_drop_target_motion else "disabled")
    )
    print(
        "Home-program call gate: "
        + ("enabled" if node.enable_home_program_call else "disabled")
    )


def _argument_parser() -> argparse.ArgumentParser:
    """Create the combined assistant CLI parser."""
    parser = argparse.ArgumentParser(
        description=(
            "Use local Ollama for guarded FANUC tasks and deterministic "
            "vision-guided XY alignment."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="permit gated execution after live checks and confirmation",
    )
    parser.add_argument("--model", default=None)
    parser.add_argument("--url", default=None)
    parser.add_argument("prompt", nargs="*")
    return parser


def _cli_arguments(args: Optional[Sequence[str]]) -> Sequence[str]:
    """Remove ROS arguments before parsing application options."""
    raw_args = list(sys.argv) if args is None else [sys.argv[0], *args]
    return remove_ros_args(args=raw_args)[1:]


def _interactive(node: FanucpyAssistant, execution_enabled: bool) -> int:
    """Run the combined human-facing conversation loop."""
    _print_help(execution_enabled)
    while rclpy.ok():
        try:
            prompt = input("You> ").strip()
        except EOFError:
            print()
            return 0
        if not prompt:
            continue
        command = prompt.lower()
        if command in {"/quit", "quit", "exit"}:
            return 0
        if command in {"/help", "help"}:
            _print_help(execution_enabled)
            continue
        if command == "/clear":
            node.ollama.clear_history()
            node.package_conversation.clear()
            print("Conversation history cleared.")
            continue
        if command == "/status":
            _print_combined_status(node)
            continue
        process_assistant_prompt(node, prompt, execution_enabled)
    return 0


def run(args: Optional[Sequence[str]] = None) -> int:
    """Run one combined request or an interactive conversation."""
    options = _argument_parser().parse_args(_cli_arguments(args))
    rclpy.init(args=args)
    node: Optional[FanucpyAssistant] = None
    try:
        node = FanucpyAssistant(
            model_override=options.model,
            url_override=options.url,
        )
        if options.prompt:
            return process_assistant_prompt(
                node,
                " ".join(options.prompt),
                options.execute,
            )
        if not sys.stdin.isatty():
            node.get_logger().error(
                "Interactive conversation requires an interactive terminal"
            )
            return 2
        return _interactive(node, options.execute)
    except (ValueError, OllamaPlannerError) as exc:
        if node is None:
            print(f"Configuration error: {exc}", file=sys.stderr)
        else:
            node.get_logger().error(str(exc))
        return 2
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Interrupted locally. This does not guarantee active robot "
                "motion has stopped; use controller HOLD or emergency stop."
            )
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: Optional[Sequence[str]] = None) -> None:
    """Console-script entry point."""
    exit_code = run(args)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
