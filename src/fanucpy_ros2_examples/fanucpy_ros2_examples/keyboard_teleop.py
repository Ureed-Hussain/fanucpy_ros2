#!/usr/bin/env python3
# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Interactive keyboard client for bounded Cartesian jog actions."""

import select
import sys
import termios
import time
import tty
from typing import Any, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fanucpy_ros2_interfaces.action import JogCartesian
from fanucpy_ros2_interfaces.msg import CartesianState, DriverStatus

from .keyboard_layout import (
    bounded_step,
    offset_for_key,
    translation_preset_for_key,
    velocity_increment,
    velocity_preset_for_key,
)


class TerminalKeyReader:
    """Temporarily place an interactive terminal in raw keyboard mode."""

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop must run in an interactive terminal")
        self._file_descriptor = sys.stdin.fileno()
        self._original_settings = termios.tcgetattr(self._file_descriptor)

    def __enter__(self) -> "TerminalKeyReader":
        tty.setraw(self._file_descriptor)
        return self

    def __exit__(self, _exc_type: Any, _exc: Any, _traceback: Any) -> None:
        termios.tcsetattr(
            self._file_descriptor,
            termios.TCSADRAIN,
            self._original_settings,
        )

    @staticmethod
    def read_key(timeout_sec: float = 0.03) -> Optional[str]:
        """Return one pressed key, or None when the timeout expires."""
        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if readable:
            return sys.stdin.read(1)
        return None


class FanucpyKeyboardTeleop(Node):
    """Send one bounded Cartesian action goal per accepted keyboard command."""

    def __init__(self) -> None:
        super().__init__("fanucpy_keyboard_teleop")

        self.declare_parameter("action_name", "/fanuc/jog_cartesian")
        self.declare_parameter("status_topic", "/fanuc/driver_status")
        self.declare_parameter("cartesian_state_topic", "/fanuc/cartesian_state")
        self.declare_parameter("frame_id", "fanuc_world")
        self.declare_parameter("translation_step_mm", 1.0)
        self.declare_parameter("rotation_step_deg", 0.5)
        self.declare_parameter("max_translation_step_mm", 50.0)
        self.declare_parameter("max_rotation_step_deg", 2.0)
        self.declare_parameter("cartesian_velocity_mm_s", 25)
        self.declare_parameter("max_cartesian_velocity_mm_s", 2000)

        action_name = str(self.get_parameter("action_name").value)
        status_topic = str(self.get_parameter("status_topic").value)
        cartesian_topic = str(self.get_parameter("cartesian_state_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.translation_step_mm = float(
            self.get_parameter("translation_step_mm").value
        )
        self.rotation_step_deg = float(
            self.get_parameter("rotation_step_deg").value
        )
        self.max_translation_step_mm = float(
            self.get_parameter("max_translation_step_mm").value
        )
        self.max_rotation_step_deg = float(
            self.get_parameter("max_rotation_step_deg").value
        )
        self.cartesian_velocity_mm_s = int(
            self.get_parameter("cartesian_velocity_mm_s").value
        )
        self.max_cartesian_velocity_mm_s = int(
            self.get_parameter("max_cartesian_velocity_mm_s").value
        )
        self._validate_parameters()

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._action_client = ActionClient(self, JogCartesian, action_name)
        self.create_subscription(
            DriverStatus,
            status_topic,
            self._status_callback,
            status_qos,
        )
        self.create_subscription(
            CartesianState,
            cartesian_topic,
            self._cartesian_state_callback,
            10,
        )

        self._armed = False
        self._driver_connected = False
        self._driver_limits_received = False
        self._motion_commands_enabled = False
        self._goal_in_flight = False
        self._latest_state: Optional[CartesianState] = None

    def _validate_parameters(self) -> None:
        if not self.frame_id.strip():
            raise ValueError("frame_id must not be empty")
        if not 0.1 <= self.translation_step_mm <= self.max_translation_step_mm:
            raise ValueError("translation_step_mm is outside the configured limits")
        if not 0.1 <= self.rotation_step_deg <= self.max_rotation_step_deg:
            raise ValueError("rotation_step_deg is outside the configured limits")
        if not 0.1 <= self.max_translation_step_mm <= 100.0:
            raise ValueError("max_translation_step_mm must be in the range 0.1..100")
        if not 0.1 <= self.max_rotation_step_deg <= 30.0:
            raise ValueError("max_rotation_step_deg must be in the range 0.1..30")
        if not 1 <= self.max_cartesian_velocity_mm_s <= 9999:
            raise ValueError(
                "max_cartesian_velocity_mm_s must be in the range 1..9999"
            )
        if not 1 <= self.cartesian_velocity_mm_s <= self.max_cartesian_velocity_mm_s:
            raise ValueError("cartesian_velocity_mm_s is outside configured limits")

    def _status_callback(self, message: DriverStatus) -> None:
        self._driver_limits_received = True
        self._motion_commands_enabled = bool(message.motion_commands_enabled)
        reported_max_velocity = int(message.max_cartesian_velocity_mm_s)
        if reported_max_velocity > 0:
            limits_changed = (
                self.max_cartesian_velocity_mm_s != reported_max_velocity
                or self.max_translation_step_mm
                != float(message.max_translation_step_mm)
                or self.max_rotation_step_deg != float(message.max_rotation_step_deg)
            )
            self.max_cartesian_velocity_mm_s = reported_max_velocity
            self.max_translation_step_mm = float(message.max_translation_step_mm)
            self.max_rotation_step_deg = float(message.max_rotation_step_deg)
            self.cartesian_velocity_mm_s = min(
                self.cartesian_velocity_mm_s,
                self.max_cartesian_velocity_mm_s,
            )
            self.translation_step_mm = min(
                self.translation_step_mm,
                self.max_translation_step_mm,
            )
            self.rotation_step_deg = min(
                self.rotation_step_deg,
                self.max_rotation_step_deg,
            )
            if limits_changed:
                self.get_logger().info(
                    f"Loaded {message.robot_model} driver limits: "
                    f"maximum Cartesian velocity "
                    f"{self.max_cartesian_velocity_mm_s} mm/s"
                )

        connected = message.state == DriverStatus.CONNECTED
        if self._driver_connected and not connected and self._armed:
            self._armed = False
            self.get_logger().warning("Driver disconnected; keyboard is now DISARMED")
        self._driver_connected = connected

    def _cartesian_state_callback(self, message: CartesianState) -> None:
        self._latest_state = message

    def print_help(self) -> None:
        """Print the keyboard layout and the non-negotiable stop warning."""
        velocity_presets = [
            velocity_preset_for_key(key, self.max_cartesian_velocity_mm_s)
            for key in ("6", "7", "8", "9", "0")
        ]
        velocity_step = velocity_increment(self.max_cartesian_velocity_mm_s)
        print(
            "\nFANUC Cartesian keyboard teleop (step mode)\n"
            "------------------------------------------------------------\n"
            "SPACE : arm/disarm future key commands\n"
            "W/S   : +Y / -Y       A/D : -X / +X\n"
            "R/F   : +Z / -Z\n"
            "U/O   : +W / -W       I/K : +P / -P\n"
            "J/L   : +R / -R\n"
            "+/-   : increase/decrease translation step\n"
            "1..5  : translation presets 1/5/10/25/50 mm\n"
            f",/.   : decrease/increase velocity by {velocity_step} mm/s\n"
            "6..0  : 10/25/50/75/100% of the robot speed limit\n"
            f"        ({'/'.join(str(value) for value in velocity_presets)} mm/s)\n"
            "]/ [  : increase/decrease rotation step\n"
            "P     : print current Cartesian state\n"
            "H     : show this help\n"
            "Q     : quit after the current move finishes\n"
            "Ctrl-C: quit immediately (does not stop controller motion)\n"
            "------------------------------------------------------------\n"
            "IMPORTANT: SPACE, Q, and Ctrl-C do not abort active FANUC motion.\n"
            "Use teach-pendant HOLD or the emergency stop when motion must stop.\n"
        )
        self._print_steps()

    def _print_steps(self) -> None:
        self.get_logger().info(
            f"Translation step: {self.translation_step_mm:.1f} mm; "
            f"rotation step: {self.rotation_step_deg:.1f} deg; "
            f"velocity: {self.cartesian_velocity_mm_s} mm/s; "
            f"keyboard: {'ARMED' if self._armed else 'DISARMED'}"
        )

    def _print_state(self) -> None:
        state = self._latest_state
        if state is None:
            self.get_logger().warning("No Cartesian state received yet")
            return
        self.get_logger().info(
            "Current [X Y Z W P R] = "
            f"[{state.x_mm:.3f}, {state.y_mm:.3f}, {state.z_mm:.3f}, "
            f"{state.w_deg:.3f}, {state.p_deg:.3f}, {state.r_deg:.3f}] "
            "[mm, deg]"
        )

    def _toggle_arm(self) -> None:
        if self._armed:
            self._armed = False
            self.get_logger().warning("Keyboard DISARMED for future commands")
            return
        if not self._driver_connected:
            self.get_logger().warning("Cannot arm: driver is not connected")
            return
        if not self._motion_commands_enabled:
            self.get_logger().warning(
                "Cannot arm: driver motion commands are disabled. Restart bringup "
                "with enable_motion_commands:=true."
            )
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warning(
                "Cannot arm: jog action server is unavailable. Restart bringup "
                "with enable_motion_commands:=true."
            )
            return
        self._armed = True
        self.get_logger().warning("Keyboard ARMED for bounded Cartesian motion")

    def _send_offset(self, offset: Sequence[float]) -> None:
        if not self._armed:
            self.get_logger().warning("Keyboard is DISARMED; press SPACE to arm")
            return
        if self._goal_in_flight:
            self.get_logger().warning("A jog is already executing; key ignored")
            return
        if not self._driver_connected or not self._action_client.server_is_ready():
            self._armed = False
            self.get_logger().warning("Driver/action unavailable; keyboard DISARMED")
            return

        goal = JogCartesian.Goal()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        (
            goal.delta_x_mm,
            goal.delta_y_mm,
            goal.delta_z_mm,
            goal.delta_w_deg,
            goal.delta_p_deg,
            goal.delta_r_deg,
        ) = offset
        goal.velocity_mm_s = self.cartesian_velocity_mm_s

        self._goal_in_flight = True
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            "Sent offset [dX dY dZ dW dP dR] = "
            f"[{', '.join(f'{value:+.3f}' for value in offset)}] [mm, deg]"
        )

    def _feedback_callback(self, feedback_message: Any) -> None:
        self.get_logger().debug(feedback_message.feedback.state)

    def _goal_response_callback(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_in_flight = False
            self.get_logger().error(f"Failed to send Cartesian jog: {exc}")
            return

        if not goal_handle.accepted:
            self._goal_in_flight = False
            self.get_logger().error(
                "Cartesian jog was rejected; check the driver terminal and limits"
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Any) -> None:
        self._goal_in_flight = False
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"Cartesian jog result failed: {exc}")
            return

        if result.success:
            self.get_logger().info(
                "Jog completed at [X Y Z W P R] = "
                f"[{result.target_x_mm:.3f}, {result.target_y_mm:.3f}, "
                f"{result.target_z_mm:.3f}, {result.target_w_deg:.3f}, "
                f"{result.target_p_deg:.3f}, {result.target_r_deg:.3f}]"
            )
        else:
            self.get_logger().error(result.message)

    def _adjust_steps(self, key: str) -> bool:
        preset = translation_preset_for_key(
            key,
            self.max_translation_step_mm,
        )
        if preset is not None:
            self.translation_step_mm = preset
            self._print_steps()
            return True

        if key in ("+", "="):
            self.translation_step_mm = bounded_step(
                self.translation_step_mm,
                1.0,
                1.0,
                self.max_translation_step_mm,
            )
        elif key == "-":
            self.translation_step_mm = bounded_step(
                self.translation_step_mm,
                -1.0,
                1.0,
                self.max_translation_step_mm,
            )
        elif key == "]":
            self.rotation_step_deg = bounded_step(
                self.rotation_step_deg,
                0.5,
                0.5,
                self.max_rotation_step_deg,
            )
        elif key == "[":
            self.rotation_step_deg = bounded_step(
                self.rotation_step_deg,
                -0.5,
                0.5,
                self.max_rotation_step_deg,
            )
        else:
            return False
        self._print_steps()
        return True

    def _adjust_velocity(self, key: str) -> bool:
        preset = velocity_preset_for_key(
            key,
            self.max_cartesian_velocity_mm_s,
        )
        if preset is not None:
            self.cartesian_velocity_mm_s = preset
            self._print_steps()
            return True

        if key == ".":
            adjustment = velocity_increment(self.max_cartesian_velocity_mm_s)
            self.cartesian_velocity_mm_s = int(
                bounded_step(
                    self.cartesian_velocity_mm_s,
                    adjustment,
                    1,
                    self.max_cartesian_velocity_mm_s,
                )
            )
        elif key == ",":
            adjustment = velocity_increment(self.max_cartesian_velocity_mm_s)
            self.cartesian_velocity_mm_s = int(
                bounded_step(
                    self.cartesian_velocity_mm_s,
                    -adjustment,
                    1,
                    self.max_cartesian_velocity_mm_s,
                )
            )
        else:
            return False
        self._print_steps()
        return True

    def _handle_key(self, key: str) -> bool:
        if key == "\x03":
            if self._goal_in_flight:
                self.get_logger().warning(
                    "Teleop is exiting, but the active controller move may continue"
                )
            return False
        if key.lower() == "q":
            if self._goal_in_flight:
                self.get_logger().warning(
                    "Wait for the active jog to finish before quitting"
                )
                return True
            return False
        if key == " ":
            self._toggle_arm()
            return True
        if key.lower() == "h":
            self.print_help()
            return True
        if key.lower() == "p":
            self._print_state()
            return True
        if self._adjust_steps(key):
            return True
        if self._adjust_velocity(key):
            return True

        offset = offset_for_key(
            key,
            self.translation_step_mm,
            self.rotation_step_deg,
        )
        if offset is not None:
            self._send_offset(offset)
        return True

    def run(self) -> None:
        """Process ROS events and terminal keys until the operator exits."""
        status_deadline = time.monotonic() + 1.0
        while (
            rclpy.ok()
            and not self._driver_limits_received
            and time.monotonic() < status_deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.print_help()
        with TerminalKeyReader() as key_reader:
            running = True
            while running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                key = key_reader.read_key(timeout_sec=0.03)
                if key is not None:
                    running = self._handle_key(key)

    def destroy_node(self) -> None:
        """Destroy the action client before destroying normal node entities."""
        self._action_client.destroy()
        super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the interactive FANUC keyboard teleop client."""
    rclpy.init(args=args)
    node: Optional[FanucpyKeyboardTeleop] = None
    try:
        node = FanucpyKeyboardTeleop()
        node.run()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
