# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Command-line ROS 2 clients for FANUC registers, I/O, power, and programs."""

import argparse
import sys
from typing import Any, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import SetBool

from fanucpy_ros2_interfaces.action import RunProgram
from fanucpy_ros2_interfaces.srv import (
    GetDigitalOutput,
    GetNumericRegister,
    GetPower,
    SetDigitalOutput,
    SetNumericRegister,
)


def boolean_value(value: str) -> bool:
    """Convert one friendly CLI boolean or raise an argparse error."""
    normalized = value.strip().lower()
    if normalized in ("true", "on", "1"):
        return True
    if normalized in ("false", "off", "0"):
        return False
    raise argparse.ArgumentTypeError("expected true/false, on/off, or 1/0")


def format_power_result(success: bool, watts: float, message: str) -> str:
    """Format power only when the controller returned a valid measurement."""
    if success:
        return f"power={watts:.3f} W; {message}"
    return message


def build_parser() -> argparse.ArgumentParser:
    """Build the student-facing controller utility command parser."""
    parser = argparse.ArgumentParser(
        description="Use FANUC controller utilities through ROS 2 interfaces.",
    )
    parser.add_argument(
        "--driver-namespace",
        default="/fanuc",
        help="Namespace containing the FANUC driver (default: /fanuc).",
    )
    parser.add_argument(
        "--wait-sec",
        type=float,
        default=10.0,
        help="Seconds to wait for a service or action server.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("power", help="Read instantaneous controller power.")

    get_register = subparsers.add_parser(
        "get-register",
        help="Read one numeric R[] register.",
    )
    get_register.add_argument("register_number", type=int)

    set_register = subparsers.add_parser(
        "set-register",
        help="Write one integer or floating-point numeric R[] register.",
    )
    set_register.add_argument("register_number", type=int)
    set_register.add_argument("value_type", choices=("integer", "float"))
    set_register.add_argument("value")

    get_output = subparsers.add_parser(
        "get-output",
        help="Read one RDO or DOUT.",
    )
    get_output.add_argument("output_type", choices=("rdo", "dout"))
    get_output.add_argument("output_number", type=int)

    set_output = subparsers.add_parser(
        "set-output",
        help="Write one RDO or DOUT.",
    )
    set_output.add_argument("output_type", choices=("rdo", "dout"))
    set_output.add_argument("output_number", type=int)
    set_output.add_argument("value", type=boolean_value)

    gripper = subparsers.add_parser(
        "gripper",
        help="Write the driver-configured gripper output.",
    )
    gripper.add_argument("value", type=boolean_value)

    run_program = subparsers.add_parser(
        "run-program",
        help="Run one enabled and allowlisted TP program.",
    )
    run_program.add_argument("program_name")
    return parser


class ControllerToolsExample(Node):
    """Execute one selected controller utility call and report its result."""

    def __init__(self, command: argparse.Namespace) -> None:
        super().__init__("fanucpy_controller_tools_example")
        namespace = "/" + command.driver_namespace.strip("/")
        self._namespace = namespace.rstrip("/")
        self._wait_sec = float(command.wait_sec)
        if self._wait_sec <= 0.0:
            raise ValueError("--wait-sec must be greater than zero")

    def _endpoint(self, name: str) -> str:
        return f"{self._namespace}/{name}"

    def _call_service(self, service_type: Any, name: str, request: Any) -> Any:
        client = self.create_client(service_type, self._endpoint(name))
        if not client.wait_for_service(timeout_sec=self._wait_sec):
            raise RuntimeError(f"Service {self._endpoint(name)} is unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def execute(self, command: argparse.Namespace) -> tuple[bool, str]:
        """Run the selected command and return success plus printable output."""
        if command.command == "power":
            response = self._call_service(GetPower, "get_power", GetPower.Request())
            return response.success, format_power_result(
                response.success,
                response.watts,
                response.message,
            )

        if command.command == "get-register":
            request = GetNumericRegister.Request()
            request.register_number = command.register_number
            response = self._call_service(
                GetNumericRegister,
                "get_numeric_register",
                request,
            )
            value = (
                response.integer_value
                if response.value_type == GetNumericRegister.Response.INTEGER
                else response.float_value
            )
            return response.success, (
                f"R[{command.register_number}]={value}; {response.message}"
            )

        if command.command == "set-register":
            request = SetNumericRegister.Request()
            request.register_number = command.register_number
            if command.value_type == "integer":
                request.value_type = SetNumericRegister.Request.INTEGER
                request.integer_value = int(command.value)
            else:
                request.value_type = SetNumericRegister.Request.FLOAT
                request.float_value = float(command.value)
            response = self._call_service(
                SetNumericRegister,
                "set_numeric_register",
                request,
            )
            return response.success, response.message

        if command.command in ("get-output", "set-output"):
            service_type = (
                GetDigitalOutput
                if command.command == "get-output"
                else SetDigitalOutput
            )
            request = service_type.Request()
            request.output_type = (
                service_type.Request.RDO
                if command.output_type == "rdo"
                else service_type.Request.DOUT
            )
            request.output_number = command.output_number
            if command.command == "set-output":
                request.value = command.value
            service_name = (
                "get_digital_output"
                if command.command == "get-output"
                else "set_digital_output"
            )
            response = self._call_service(service_type, service_name, request)
            prefix = (
                f"{command.output_type.upper()}[{command.output_number}]="
                f"{response.value}; "
                if command.command == "get-output"
                else ""
            )
            return response.success, prefix + response.message

        if command.command == "gripper":
            request = SetBool.Request()
            request.data = command.value
            response = self._call_service(SetBool, "set_gripper", request)
            return response.success, response.message

        if command.command == "run-program":
            return self._run_program(command.program_name)

        raise ValueError(f"Unsupported command: {command.command}")

    def _run_program(self, program_name: str) -> tuple[bool, str]:
        action_name = self._endpoint("run_program")
        client = ActionClient(self, RunProgram, action_name)
        if not client.wait_for_server(timeout_sec=self._wait_sec):
            raise RuntimeError(f"Action {action_name} is unavailable")
        goal = RunProgram.Goal()
        goal.program_name = program_name
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, f"TP program {program_name} was rejected"
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, "TP program result was unavailable"
        result = wrapped_result.result
        return result.success, result.message


def main(args: Optional[Sequence[str]] = None) -> int:
    """Run one command selected on the command line."""
    argv = list(sys.argv if args is None else args)
    command = build_parser().parse_args(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node: Optional[ControllerToolsExample] = None
    try:
        node = ControllerToolsExample(command)
        success, message = node.execute(command)
        print(message)
        return 0 if success else 1
    except (RuntimeError, ValueError) as exc:
        print(f"Controller utility failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
