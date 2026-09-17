#!/usr/bin/env python3
# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Conversational local-Ollama client for bounded FANUC ROS 2 tasks."""

import argparse
import math
import sys
import time
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.utilities import remove_ros_args

from fanucpy_ros2_interfaces.msg import (
    CartesianState,
    DriverStatus,
    VisionDetectionArray,
)
from sensor_msgs.msg import JointState

from .conversation import (
    CartesianSnapshot,
    JointSnapshot,
    format_absolute_cartesian_target,
    format_partial_cartesian_target,
    format_joint_snapshot,
    format_joint_target,
    format_snapshot,
    format_target,
    model_robot_context,
)
from .ollama_client import ModelDecision, OllamaPlanner, OllamaPlannerError
from .prompt_control import FanucpyPromptControl
from .prompt_parser import CartesianJogPlan, PromptParseError, format_plan
from .task_plans import (
    CartesianTargetPlan,
    JointTargetPlan,
    TaskPlan,
    TpProgramPlan,
    resolve_cartesian_target,
    validate_joint_target_plan,
)
from .vision_context import (
    VisionObservation,
    coordinate_question_answer,
    is_vision_observation_request,
    model_vision_context,
)


class FanucpyOllamaPrompt(FanucpyPromptControl):
    """Combine live robot context with a schema-constrained local model."""

    def __init__(
        self,
        model_override: Optional[str] = None,
        url_override: Optional[str] = None,
        node_name: str = "fanucpy_ollama_prompt",
    ) -> None:
        super().__init__(node_name=node_name)
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434")
        self.declare_parameter("ollama_model", "qwen3:8b")
        self.declare_parameter("ollama_timeout_sec", 60.0)
        self.declare_parameter("ollama_history_turns", 6)
        self.declare_parameter("allow_remote_ollama", False)
        self.declare_parameter("robot_context_timeout_sec", 2.0)
        self.declare_parameter(
            "vision_detections_topic",
            "/fanuc/vision/detections",
        )
        self.declare_parameter("max_vision_state_age_sec", 1.0)

        model = model_override or str(self.get_parameter("ollama_model").value)
        base_url = url_override or str(self.get_parameter("ollama_url").value)
        self.robot_context_timeout_sec = float(
            self.get_parameter("robot_context_timeout_sec").value
        )
        if self.robot_context_timeout_sec <= 0.0:
            raise ValueError(
                "robot_context_timeout_sec must be greater than zero"
            )
        self.max_vision_state_age_sec = float(
            self.get_parameter("max_vision_state_age_sec").value
        )
        if self.max_vision_state_age_sec <= 0.0:
            raise ValueError(
                "max_vision_state_age_sec must be greater than zero"
            )
        vision_topic = str(
            self.get_parameter("vision_detections_topic").value
        ).strip()
        if not vision_topic:
            raise ValueError("vision_detections_topic cannot be empty")
        self._latest_vision: Optional[VisionDetectionArray] = None
        self._vision_received_at = 0.0
        self._vision_subscription = self.create_subscription(
            VisionDetectionArray,
            vision_topic,
            self._vision_callback,
            10,
        )

        self.ollama = OllamaPlanner(
            parser=self.parser,
            base_url=base_url,
            model=model,
            timeout_sec=float(self.get_parameter("ollama_timeout_sec").value),
            history_turns=int(
                self.get_parameter("ollama_history_turns").value
            ),
            allow_remote=bool(
                self.get_parameter("allow_remote_ollama").value
            ),
            joint_lower_limits_rad=self.joint_lower_limits_rad,
            joint_upper_limits_rad=self.joint_upper_limits_rad,
            max_joint_velocity_percent=self.max_joint_velocity_percent,
        )

    @property
    def model_name(self) -> str:
        """Return the configured local Ollama model name."""
        return self.ollama.model

    @property
    def latest_vision(self) -> Optional[VisionDetectionArray]:
        """Return the most recently received normalized vision batch."""
        return self._latest_vision

    @property
    def vision_received_at(self) -> float:
        """Return the monotonic receipt time of the latest vision batch."""
        return self._vision_received_at

    def _vision_callback(self, message: VisionDetectionArray) -> None:
        self._latest_vision = message
        self._vision_received_at = time.monotonic()

    def wait_for_vision(
        self,
        timeout_sec: Optional[float] = None,
        received_after: float = 0.0,
    ) -> Optional[VisionDetectionArray]:
        """Spin until a fresh normalized vision batch is available."""
        timeout = (
            self.robot_context_timeout_sec
            if timeout_sec is None
            else max(0.0, timeout_sec)
        )
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            message = self._latest_vision
            age_sec = time.monotonic() - self._vision_received_at
            if (
                message is not None
                and age_sec <= self.max_vision_state_age_sec
                and self._vision_received_at >= received_after
            ):
                return message
            if time.monotonic() >= deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    @staticmethod
    def vision_observations(
        message: VisionDetectionArray,
    ) -> Tuple[VisionObservation, ...]:
        """Copy one typed ROS batch into pure immutable observations."""
        return tuple(
            VisionObservation(
                track_id=int(item.track_id),
                label=item.label,
                is_danger=bool(item.is_danger),
                identity_source=item.identity_source,
                projected_center_valid=bool(item.projected_center_valid),
                projected_u_px=float(item.projected_u_px),
                projected_v_px=float(item.projected_v_px),
                eye_to_hand_valid=bool(item.eye_to_hand_valid),
                eye_to_hand_x_mm=float(item.eye_to_hand_x_mm),
                eye_to_hand_y_mm=float(item.eye_to_hand_y_mm),
            )
            for item in message.detections
        )

    def vision_context(self) -> str:
        """Return fresh normalized detections as trusted model context."""
        message = self._latest_vision
        if message is None:
            return model_vision_context(
                observations=None,
                age_sec=math.inf,
                maximum_age_sec=self.max_vision_state_age_sec,
            )
        age_sec = time.monotonic() - self._vision_received_at
        return model_vision_context(
            observations=self.vision_observations(message),
            age_sec=age_sec,
            maximum_age_sec=self.max_vision_state_age_sec,
            source=message.source,
            frame_sequence=message.frame_sequence,
            eye_to_hand_frame_id=message.eye_to_hand_frame_id,
            eye_to_hand_calibration=message.eye_to_hand_calibration,
        )

    def coordinate_answer(self, prompt: str) -> Optional[str]:
        """Answer an object-coordinate question from typed ROS data only."""
        message = self._latest_vision
        observations = (
            None
            if message is None
            else self.vision_observations(message)
        )
        age_sec = (
            math.inf
            if message is None
            else time.monotonic() - self._vision_received_at
        )
        return coordinate_question_answer(
            prompt=prompt,
            observations=observations,
            age_sec=age_sec,
            maximum_age_sec=self.max_vision_state_age_sec,
            eye_to_hand_frame_id=(
                message.eye_to_hand_frame_id if message is not None else ""
            ),
            eye_to_hand_calibration=(
                message.eye_to_hand_calibration if message is not None else ""
            ),
        )

    @staticmethod
    def cartesian_snapshot(message: CartesianState) -> CartesianSnapshot:
        frame_id = message.header.frame_id.strip() or "unlabelled"
        return CartesianSnapshot(
            frame_id=frame_id,
            x_mm=float(message.x_mm),
            y_mm=float(message.y_mm),
            z_mm=float(message.z_mm),
            w_deg=float(message.w_deg),
            p_deg=float(message.p_deg),
            r_deg=float(message.r_deg),
        )

    @staticmethod
    def _driver_state(status: Optional[DriverStatus]) -> str:
        if status is None:
            return "UNAVAILABLE"
        names = {
            DriverStatus.DISCONNECTED: "DISCONNECTED",
            DriverStatus.CONNECTING: "CONNECTING",
            DriverStatus.CONNECTED: "CONNECTED",
            DriverStatus.ERROR: "ERROR",
        }
        return names.get(status.state, f"UNKNOWN({status.state})")

    def joint_snapshot(self, message: JointState) -> JointSnapshot:
        positions = self.ordered_joint_positions(message)
        if not all(math.isfinite(value) for value in positions):
            raise RuntimeError("Joint state contains non-finite positions")
        return JointSnapshot(
            names=self.joint_names,  # type: ignore[arg-type]
            positions_rad=positions,
        )

    def robot_context(
        self,
        require_fresh_state: bool = True,
    ) -> Tuple[
        str,
        Optional[CartesianSnapshot],
        Optional[JointSnapshot],
    ]:
        """Collect current state and format trusted model context."""
        state = self.wait_for_cartesian_state(
            timeout_sec=self.robot_context_timeout_sec,
            require_fresh=require_fresh_state,
        )
        snapshot = (
            self.cartesian_snapshot(state) if state is not None else None
        )
        joint_state = self.wait_for_joint_state(
            timeout_sec=self.robot_context_timeout_sec,
            require_fresh=require_fresh_state,
        )
        joint_snapshot = (
            self.joint_snapshot(joint_state)
            if joint_state is not None
            else None
        )
        status = self.latest_status

        max_translation = self.parser.max_translation_step_mm
        max_rotation = self.parser.max_rotation_step_deg
        max_velocity = self.parser.max_cartesian_velocity_mm_s
        motion_enabled = False
        if status is not None:
            motion_enabled = bool(status.motion_commands_enabled)
            if status.max_translation_step_mm > 0.0:
                max_translation = float(status.max_translation_step_mm)
            if status.max_rotation_step_deg > 0.0:
                max_rotation = float(status.max_rotation_step_deg)
            if status.max_cartesian_velocity_mm_s > 0:
                max_velocity = int(status.max_cartesian_velocity_mm_s)

        context = model_robot_context(
            snapshot=snapshot,
            driver_state=self._driver_state(status),
            motion_enabled=motion_enabled,
            frame_id=self.frame_id,
            max_translation_step_mm=max_translation,
            max_rotation_step_deg=max_rotation,
            max_velocity_mm_s=max_velocity,
            joint_snapshot=joint_snapshot,
            max_joint_delta_rad=self.max_joint_command_delta_rad,
            default_joint_velocity_percent=(
                int(status.default_joint_velocity_percent)
                if status is not None
                and status.default_joint_velocity_percent > 0
                else self.default_joint_velocity_percent
            ),
            max_joint_velocity_percent=(
                int(status.max_joint_velocity_percent)
                if status is not None and status.max_joint_velocity_percent > 0
                else self.max_joint_velocity_percent
            ),
            absolute_cartesian_enabled=(
                bool(status.absolute_cartesian_commands_enabled)
                if status is not None
                else False
            ),
            absolute_cartesian_bounds_enabled=(
                bool(status.absolute_cartesian_bounds_enabled)
                if status is not None
                else False
            ),
            program_execution_enabled=(
                bool(status.program_execution_enabled)
                if status is not None
                else False
            ),
            allowed_tp_programs=(
                tuple(status.allowed_tp_programs)
                if status is not None
                else ()
            ),
        ) + "\n" + self.vision_context()
        return context, snapshot, joint_snapshot


def print_help(execution_enabled: bool) -> None:
    mode = (
        "execution permitted with confirmation"
        if execution_enabled
        else "dry run"
    )
    print(
        "\nLocal Ollama FANUC conversation\n"
        "--------------------------------\n"
        f"Mode: {mode}\n"
        "Examples:\n"
        "  please move to the left side by 5 mm at 20 mm/s\n"
        "  move X 300\n"
        "  set X to -350 and Z to -190 at 100 mm/s\n"
        "  move to X -350 Y 800 Z -190 W -179 P 60 R -175 at 100 mm/s\n"
        "  move joints to J1 0 J2 0 J3 0 J4 0 J5 0 J6 0 at 5%\n"
        "  run TP program HOME_P\n"
        "  go a little upward\n"
        "  rotate yaw positive by 0.5 degrees\n"
        "  what is the current robot position?\n"
        "  what do you see?\n"
        "  how many batteries can you see?\n"
        "  tell me the coordinates of battery one\n"
        "  where is bottle 1?\n"
        "  give me the projected pixel of track 12\n"
        "  do the same movement again\n"
        "Commands:\n"
        "  /status  show live state without calling the model\n"
        "  /clear   clear model conversation history\n"
        "  /help    show this help\n"
        "  /quit    exit\n"
    )


def print_status(node: FanucpyOllamaPrompt) -> None:
    _, snapshot, joint_snapshot = node.robot_context(require_fresh_state=True)
    status = node.latest_status
    print(f"Driver: {node._driver_state(status)}")
    print(
        "Motion gate: "
        + (
            "enabled"
            if status is not None and status.motion_commands_enabled
            else "disabled or unavailable"
        )
    )
    if snapshot is None:
        print("Current Cartesian position: unavailable")
    else:
        print(format_snapshot(snapshot))
    if joint_snapshot is None:
        print("Current joint position: unavailable")
    else:
        print(format_joint_snapshot(joint_snapshot))
    if status is not None:
        print(
            "Direct absolute Cartesian gate: "
            + (
                "enabled"
                if status.absolute_cartesian_commands_enabled
                else "disabled"
            )
        )
        print(
            "TP program gate: "
            + ("enabled" if status.program_execution_enabled else "disabled")
        )
        print(f"Allowed TP programs: {list(status.allowed_tp_programs)}")


def _print_decision(
    node: FanucpyOllamaPrompt,
    decision: ModelDecision,
    snapshot: Optional[CartesianSnapshot],
    joint_snapshot: Optional[JointSnapshot],
) -> bool:
    print(f"\nAssistant: {decision.message}")
    if decision.needs_clarification:
        print(f"Clarification required: {decision.clarification_question}")
    if decision.plan is None:
        return True

    plan = decision.plan
    if isinstance(plan, CartesianJogPlan):
        if snapshot is None:
            print("Current Cartesian position: unavailable")
        else:
            print(format_snapshot(snapshot))
        print(format_plan(plan, node.frame_id))
        print(format_target(snapshot, plan))
        return True

    if isinstance(plan, CartesianTargetPlan):
        if not plan.is_complete:
            print(format_partial_cartesian_target(node.frame_id, plan))
        if snapshot is None:
            print("Current Cartesian position: unavailable")
            if plan.is_complete:
                print(format_absolute_cartesian_target(node.frame_id, plan))
            else:
                print(
                    "Resolved target is unavailable; omitted axes require "
                    "fresh robot feedback."
                )
            print("Travel from current pose is unavailable without feedback.")
            return True
        print(format_snapshot(snapshot))
        try:
            resolved_plan = resolve_cartesian_target(plan, snapshot.values)
        except PromptParseError as exc:
            print(f"LOCAL VALIDATION BLOCKED: {exc}")
            return False
        print(format_absolute_cartesian_target(node.frame_id, resolved_plan))
        travel = tuple(
            target - current
            for target, current in zip(resolved_plan.target, snapshot.values)
        )
        print(
            "Direct travel [dX dY dZ dW dP dR]: ["
            + ", ".join(f"{value:+.3f}" for value in travel)
            + "] [mm, deg]"
        )
        print(
            "Absolute target: relative jog-step limits do not apply; no "
            "intermediate path or collision check is generated."
        )
        return True

    if isinstance(plan, JointTargetPlan):
        if joint_snapshot is None:
            print("Current joint position: unavailable")
        else:
            print(format_joint_snapshot(joint_snapshot))
        print(format_joint_target(node.joint_names, plan))
        print(
            "Joint velocity: "
            + (
                f"{plan.velocity_percent}%"
                if plan.velocity_percent > 0
                else "driver default"
            )
        )
        if joint_snapshot is None:
            print(
                "Joint-delta validation is unavailable without joint feedback."
            )
            return True
        maximum = node.max_joint_velocity_percent
        status = node.latest_status
        if status is not None and status.max_joint_velocity_percent > 0:
            maximum = int(status.max_joint_velocity_percent)
        try:
            target_rad = validate_joint_target_plan(
                plan,
                node.joint_lower_limits_rad,
                node.joint_upper_limits_rad,
                maximum,
                current_positions_rad=joint_snapshot.positions_rad,
                max_delta_rad=node.max_joint_command_delta_rad,
            )
        except PromptParseError as exc:
            print(f"LOCAL VALIDATION BLOCKED: {exc}")
            return False
        deltas_deg = tuple(
            math.degrees(target - current)
            for target, current in zip(
                target_rad,
                joint_snapshot.positions_rad,
            )
        )
        print(
            "Joint delta [J1 J2 J3 J4 J5 J6]: ["
            + ", ".join(f"{value:+.3f}" for value in deltas_deg)
            + "] deg"
        )
        return True

    assert isinstance(plan, TpProgramPlan)
    status = node.latest_status
    print(f"TP program request: {plan.program_name}")
    if status is None:
        print("Live TP gate and allowlist are unavailable.")
        return True
    allowed = tuple(str(name).upper() for name in status.allowed_tp_programs)
    print(
        "TP program gate: "
        + ("enabled" if status.program_execution_enabled else "disabled")
    )
    print(f"Allowed TP programs: {list(allowed)}")
    if plan.program_name not in allowed:
        print("LOCAL VALIDATION BLOCKED: program is not in the live allowlist")
        return False
    if not status.program_execution_enabled:
        print(
            "DRY-RUN ONLY: the live TP program-execution gate is disabled."
        )
    return True


def _confirm_execution(token: str, description: str) -> bool:
    if not sys.stdin.isatty():
        print("Execution blocked: an interactive terminal is required")
        return False
    print(
        "Review the current state, target, active frames, tool, safeguarded "
        "workspace, controller mode, and speed."
    )
    return input(f"Type {token} to {description}: ") == token


def _show_cartesian_result(
    node: FanucpyOllamaPrompt,
    started_at: float,
) -> None:
    result_state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
        received_after=started_at,
    )
    if result_state is not None:
        print("Actual Cartesian state after action result")
        print(format_snapshot(node.cartesian_snapshot(result_state)))


def _execute_cartesian_jog(
    node: FanucpyOllamaPrompt,
    plan: CartesianJogPlan,
) -> bool:
    try:
        node.check_execution_ready(plan)
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(f"Execution blocked: {exc}")
        return False

    state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
    )
    if state is None:
        node.get_logger().error(
            "Execution blocked: no fresh Cartesian state is available"
        )
        return False
    snapshot = node.cartesian_snapshot(state)
    print("\nExecution-time robot state")
    print(format_snapshot(snapshot))
    print(format_target(snapshot, plan))
    if not _confirm_execution("EXECUTE", "send this Cartesian jog"):
        print("Motion cancelled; nothing was sent.")
        return False

    started_at = time.monotonic()
    if not node.execute(plan):
        return False
    _show_cartesian_result(node, started_at)
    return True


def _execute_cartesian_target(
    node: FanucpyOllamaPrompt,
    plan: CartesianTargetPlan,
) -> bool:
    state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
    )
    if state is None:
        node.get_logger().error(
            "Execution blocked: no fresh Cartesian state is available"
        )
        return False
    snapshot = node.cartesian_snapshot(state)
    try:
        resolved_plan = resolve_cartesian_target(plan, snapshot.values)
        node.check_absolute_cartesian_execution_ready(resolved_plan)
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(f"Execution blocked: {exc}")
        return False
    print("\nPre-confirmation absolute Cartesian target")
    if not plan.is_complete:
        print(format_partial_cartesian_target(node.frame_id, plan))
    print(format_snapshot(snapshot))
    print(format_absolute_cartesian_target(node.frame_id, resolved_plan))
    travel = tuple(
        target - current
        for target, current in zip(resolved_plan.target, snapshot.values)
    )
    print(
        "Direct travel [dX dY dZ dW dP dR]: ["
        + ", ".join(f"{value:+.3f}" for value in travel)
        + "] [mm, deg]"
    )
    print(
        "This is one direct absolute controller command. Relative jog-step "
        "limits and collision-path checking do not apply."
    )
    if not _confirm_execution(
        "EXECUTE",
        "send this direct absolute target",
    ):
        print("Motion cancelled; nothing was sent.")
        return False

    state = node.wait_for_cartesian_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
    )
    if state is None:
        node.get_logger().error(
            "Execution blocked after confirmation: Cartesian state was lost"
        )
        return False
    snapshot = node.cartesian_snapshot(state)
    try:
        resolved_plan = resolve_cartesian_target(plan, snapshot.values)
        node.check_absolute_cartesian_execution_ready(resolved_plan)
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(
            f"Execution blocked after confirmation; target was not sent: {exc}"
        )
        return False
    print("Final execution-time Cartesian state")
    print(format_snapshot(snapshot))
    print(format_absolute_cartesian_target(node.frame_id, resolved_plan))
    started_at = time.monotonic()
    if not node.execute_absolute_cartesian(resolved_plan):
        return False
    _show_cartesian_result(node, started_at)
    return True


def _execute_joint_target(
    node: FanucpyOllamaPrompt,
    plan: JointTargetPlan,
) -> bool:
    state = node.wait_for_joint_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
    )
    if state is None:
        node.get_logger().error(
            "Execution blocked: no fresh joint state is available"
        )
        return False
    try:
        snapshot = node.joint_snapshot(state)
        target, velocity = node.check_joint_execution_ready(
            plan,
            snapshot.positions_rad,
        )
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(f"Execution blocked: {exc}")
        return False
    print("\nPre-confirmation joint target")
    print(format_joint_snapshot(snapshot))
    print(format_joint_target(node.joint_names, plan))
    print(f"Resolved joint velocity: {velocity}%")
    if not _confirm_execution("EXECUTE", "send this absolute joint target"):
        print("Motion cancelled; nothing was sent.")
        return False

    state = node.wait_for_joint_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
    )
    if state is None:
        node.get_logger().error(
            "Execution blocked after confirmation: joint state was lost"
        )
        return False
    try:
        snapshot = node.joint_snapshot(state)
        target, velocity = node.check_joint_execution_ready(
            plan,
            snapshot.positions_rad,
        )
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(
            f"Execution blocked after confirmation; target was not sent: {exc}"
        )
        return False
    started_at = time.monotonic()
    if not node.execute_joint_target(
        plan,
        snapshot.positions_rad,
        target,
        velocity,
    ):
        return False
    result_state = node.wait_for_joint_state(
        timeout_sec=node.robot_context_timeout_sec,
        require_fresh=True,
        received_after=started_at,
    )
    if result_state is not None:
        print("Actual joint state after action result")
        print(format_joint_snapshot(node.joint_snapshot(result_state)))
    return True


def _execute_tp_program(
    node: FanucpyOllamaPrompt,
    plan: TpProgramPlan,
) -> bool:
    try:
        plan = node.check_program_execution_ready(plan)
    except (PromptParseError, RuntimeError) as exc:
        node.get_logger().error(f"Execution blocked: {exc}")
        return False
    print(f"\nAllowlisted TP program: {plan.program_name}")
    token = f"RUN {plan.program_name}"
    if not _confirm_execution(token, "call this TP program once"):
        print("TP program cancelled; nothing was sent.")
        return False
    return node.execute_tp_program(plan)


def _execute_decision(
    node: FanucpyOllamaPrompt,
    plan: TaskPlan,
) -> bool:
    if isinstance(plan, CartesianJogPlan):
        return _execute_cartesian_jog(node, plan)
    if isinstance(plan, CartesianTargetPlan):
        return _execute_cartesian_target(node, plan)
    if isinstance(plan, JointTargetPlan):
        return _execute_joint_target(node, plan)
    assert isinstance(plan, TpProgramPlan)
    return _execute_tp_program(node, plan)


def process_prompt(
    node: FanucpyOllamaPrompt,
    prompt: str,
    execution_enabled: bool,
) -> int:
    context, snapshot, joint_snapshot = node.robot_context(
        require_fresh_state=True
    )
    coordinate_answer = node.coordinate_answer(prompt)
    if coordinate_answer is not None:
        print(f"\nAssistant: {coordinate_answer}")
        return 0
    print(f"\nConsulting local model '{node.model_name}'...")
    try:
        decision = node.ollama.interpret(prompt, context)
    except OllamaPlannerError as exc:
        node.get_logger().error(f"Local model request failed: {exc}")
        return 2

    if is_vision_observation_request(prompt) and decision.plan is not None:
        print(f"\nAssistant: {decision.message}")
        print(
            "LOCAL VALIDATION BLOCKED: a vision-observation question cannot "
            "produce a robot command"
        )
        return 2

    preview_valid = _print_decision(
        node,
        decision,
        snapshot,
        joint_snapshot,
    )
    if decision.plan is None:
        return 0
    if not preview_valid:
        return 2
    if not execution_enabled:
        print(
            "DRY RUN: the model plan was validated but not sent. Restart with "
            "--execute to permit confirmation and execution."
        )
        return 0
    return 0 if _execute_decision(node, decision.plan) else 1


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Use a local Ollama conversation to create validated FANUC "
            "Cartesian, joint, and TP-program plans."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "permit one-at-a-time execution after live checks and confirmation"
        ),
    )
    parser.add_argument(
        "--model",
        default=None,
        help="override the ollama_model ROS parameter",
    )
    parser.add_argument(
        "--url",
        default=None,
        help="override the local ollama_url ROS parameter",
    )
    parser.add_argument(
        "prompt",
        nargs="*",
        help=(
            "optional one-shot prompt; omit it for an interactive conversation"
        ),
    )
    return parser


def _cli_arguments(args: Optional[Sequence[str]]) -> Sequence[str]:
    raw_args = list(sys.argv) if args is None else [sys.argv[0], *args]
    return remove_ros_args(args=raw_args)[1:]


def _interactive(node: FanucpyOllamaPrompt, execution_enabled: bool) -> int:
    print_help(execution_enabled)
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
            print_help(execution_enabled)
            continue
        if command == "/clear":
            node.ollama.clear_history()
            print("Conversation history cleared.")
            continue
        if command == "/status":
            print_status(node)
            continue
        process_prompt(node, prompt, execution_enabled)
    return 0


def run(args: Optional[Sequence[str]] = None) -> int:
    """Run one local-model request or an interactive conversation."""
    options = _argument_parser().parse_args(_cli_arguments(args))
    rclpy.init(args=args)
    node: Optional[FanucpyOllamaPrompt] = None
    try:
        node = FanucpyOllamaPrompt(
            model_override=options.model,
            url_override=options.url,
        )
        if options.prompt:
            return process_prompt(
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
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"Configuration error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Interrupted locally. This does not guarantee active robot "
                "motion has stopped; use controller HOLD or emergency stop "
                "when required."
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
