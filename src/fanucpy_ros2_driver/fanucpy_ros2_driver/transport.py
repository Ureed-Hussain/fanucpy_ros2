# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

"""Thread-safe, source-controlled adapter around fanucpy."""

from dataclasses import dataclass
import math
import numbers
import re
import threading
import time
from typing import Any, Callable, Optional, Sequence, Tuple, Union


Numeric = Union[int, float]
RobotFactory = Callable[..., Any]


class FanucpyDependencyError(RuntimeError):
    """Raised when the fanucpy Python dependency is unavailable."""


class FanucpyTransportError(RuntimeError):
    """Raised when a transport operation is invalid or unavailable."""


@dataclass(frozen=True)
class RobotStateSnapshot:
    """One coherent state sample read through a serialized connection."""

    joints_deg: Tuple[float, ...]
    cartesian_mm_deg: Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class CartesianMotionResult:
    """Target pose and controller response for one Cartesian movement."""

    target_mm_deg: Tuple[float, float, float, float, float, float]
    response_code: int
    response_message: str


@dataclass(frozen=True)
class JointMotionResult:
    """Target, measured joints, and controller response for one joint move."""

    target_deg: Tuple[float, float, float, float, float, float]
    actual_deg: Tuple[float, float, float, float, float, float]
    response_code: int
    response_message: str


@dataclass(frozen=True)
class ProgramCallResult:
    """TP-program response plus the state of its controller socket session."""

    response_code: int
    response_message: str
    connection_ready: bool
    recovery_message: str
    recovered_state: Optional[RobotStateSnapshot]


class FanucpyTransport:
    """
    Own and serialize access to one ``fanucpy.Robot`` instance.

    Register methods preserve the local extension that was previously added
    directly to installed copies of ``fanucpy/robot.py``. Keeping the extension
    here prevents different Python environments from carrying different
    controller behavior.
    """

    _PROGRAM_NAME = re.compile(r"^[A-Z][A-Z0-9_]{0,31}$")
    _INTEGER_REGISTER_MIN = -(2**31)
    _INTEGER_REGISTER_MAX = 2**31 - 1

    def __init__(
        self,
        robot_model: str,
        host: str,
        port: int = 18735,
        socket_timeout_sec: float = 5.0,
        ee_do_type: Optional[str] = "RDO",
        ee_do_num: Optional[int] = 7,
        robot_factory: Optional[RobotFactory] = None,
    ) -> None:
        if not host.strip():
            raise ValueError("Robot host must not be empty")
        if not 1 <= int(port) <= 65535:
            raise ValueError("Robot port must be in the range 1..65535")
        if float(socket_timeout_sec) <= 0.0:
            raise ValueError("Socket timeout must be greater than zero")

        self.robot_model = str(robot_model)
        self.host = host.strip()
        self.port = int(port)
        self.socket_timeout_sec = float(socket_timeout_sec)
        self.ee_do_type = ee_do_type
        self.ee_do_num = ee_do_num
        self._robot_factory = robot_factory
        self._robot: Optional[Any] = None
        self._connected = False
        self._lock = threading.RLock()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def _resolve_robot_factory(self) -> RobotFactory:
        if self._robot_factory is not None:
            return self._robot_factory

        try:
            from fanucpy import Robot
        except ImportError as exc:
            raise FanucpyDependencyError(
                "fanucpy is not installed for the Python interpreter used by "
                "ROS 2. Install fanucpy for system Python 3.10."
            ) from exc

        return Robot

    def connect(self) -> Tuple[int, str]:
        """Connect to the controller and return the MAPPDK response."""
        with self._lock:
            return self._connect_unlocked()

    def _connect_unlocked(self) -> Tuple[int, str]:
        if self._connected:
            return 0, "already connected"

        factory = self._resolve_robot_factory()
        kwargs = {
            "robot_model": self.robot_model,
            "host": self.host,
            "port": self.port,
            "socket_timeout": self.socket_timeout_sec,
            "ee_DO_type": self.ee_do_type,
            "ee_DO_num": self.ee_do_num,
        }
        robot = factory(**kwargs)
        self._robot = robot

        try:
            result = robot.connect()
        except Exception:
            try:
                robot.disconnect()
            except Exception:
                pass
            self._robot = None
            self._connected = False
            raise

        self._connected = True
        return result

    def disconnect(self) -> None:
        """Disconnect safely; repeated calls are allowed."""
        with self._lock:
            self._disconnect_unlocked()

    def _disconnect_unlocked(self) -> None:
        robot = self._robot
        self._robot = None
        self._connected = False

        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass

    def _require_robot(self) -> Any:
        if not self._connected or self._robot is None:
            raise FanucpyTransportError("Robot is not connected")
        return self._robot

    @staticmethod
    def _finite_values(values: Sequence[Numeric], expected: int) -> Tuple[float, ...]:
        if len(values) < expected:
            raise FanucpyTransportError(
                f"Expected at least {expected} values, received {len(values)}"
            )
        converted = tuple(float(value) for value in values[:expected])
        if not all(math.isfinite(value) for value in converted):
            raise FanucpyTransportError("Robot returned a non-finite state value")
        return converted

    def read_state(self) -> RobotStateSnapshot:
        """Read joints and Cartesian position without command interleaving."""
        with self._lock:
            return self._read_state_unlocked()

    def _read_state_unlocked(self) -> RobotStateSnapshot:
        robot = self._require_robot()
        joints = self._finite_values(robot.get_curjpos(), expected=6)
        cartesian = self._finite_values(robot.get_curpos(), expected=6)
        return RobotStateSnapshot(
            joints_deg=joints,
            cartesian_mm_deg=cartesian,  # type: ignore[arg-type]
        )

    def jog_cartesian(
        self,
        offset_mm_deg: Sequence[Numeric],
        velocity_mm_s: int,
        acceleration_percent: int,
    ) -> CartesianMotionResult:
        """
        Read the live pose and execute one relative Cartesian movement.

        The pose read and movement command share one lock, preventing state
        polling or another controller command from interleaving between them.
        """
        if len(offset_mm_deg) != 6:
            raise ValueError("A Cartesian jog requires exactly six offsets")
        offset = tuple(float(value) for value in offset_mm_deg)
        if not all(math.isfinite(value) for value in offset):
            raise ValueError("Cartesian jog offsets must be finite")
        if not 1 <= int(velocity_mm_s) <= 9999:
            raise ValueError("Cartesian velocity must be in the range 1..9999 mm/s")
        if not 1 <= int(acceleration_percent) <= 100:
            raise ValueError("Acceleration must be in the range 1..100 percent")

        with self._lock:
            robot = self._require_robot()
            current = self._finite_values(robot.get_curpos(), expected=6)
            target = tuple(
                current_value + offset_value
                for current_value, offset_value in zip(current, offset)
            )
            code, message = robot.move(
                "pose",
                list(target),
                velocity=int(velocity_mm_s),
                acceleration=int(acceleration_percent),
                cnt_val=0,
                linear=True,
            )
            return CartesianMotionResult(
                target_mm_deg=target,  # type: ignore[arg-type]
                response_code=int(code),
                response_message=str(message),
            )

    def move_joint(
        self,
        positions_deg: Sequence[Numeric],
        velocity_percent: int,
        acceleration_percent: int,
    ) -> JointMotionResult:
        """Execute one exact-stop FANUC joint waypoint and read its final state."""
        if len(positions_deg) != 6:
            raise ValueError("A FANUC joint move requires exactly six positions")
        target = tuple(float(value) for value in positions_deg)
        if not all(math.isfinite(value) for value in target):
            raise ValueError("Joint target positions must be finite")
        if not 1 <= int(velocity_percent) <= 100:
            raise ValueError("Joint velocity must be in the range 1..100 percent")
        if not 1 <= int(acceleration_percent) <= 100:
            raise ValueError(
                "Joint acceleration must be in the range 1..100 percent"
            )

        with self._lock:
            robot = self._require_robot()
            code, message = robot.move(
                "joint",
                list(target),
                velocity=int(velocity_percent),
                acceleration=int(acceleration_percent),
                cnt_val=0,
                linear=False,
            )
            actual = self._finite_values(robot.get_curjpos(), expected=6)
            return JointMotionResult(
                target_deg=target,  # type: ignore[arg-type]
                actual_deg=actual,  # type: ignore[arg-type]
                response_code=int(code),
                response_message=str(message),
            )

    @staticmethod
    def _validate_register_index(register: int) -> int:
        register = int(register)
        if not 1 <= register <= 999:
            raise ValueError("Numeric register index must be in the range 1..999")
        return register

    def set_numeric_register(self, register: int, value: Numeric) -> Tuple[int, str]:
        """Write a FANUC numeric register, including NumPy scalar values."""
        register = self._validate_register_index(register)
        if isinstance(value, bool):
            raise TypeError("Boolean values are not numeric register values")

        if isinstance(value, numbers.Integral):
            integer_value = int(value)
            if not (
                self._INTEGER_REGISTER_MIN
                <= integer_value
                <= self._INTEGER_REGISTER_MAX
            ):
                raise ValueError("Integer register value exceeds signed 32-bit range")
            command = f"setregint:{register:03}:{integer_value}"
        elif isinstance(value, numbers.Real):
            numeric_value = float(value)
            if not math.isfinite(numeric_value):
                raise ValueError("Numeric register value must be finite")
            command = f"setregflt:{register:03}:{numeric_value}"
        else:
            raise TypeError(f"Unsupported numeric register value: {type(value)!r}")

        with self._lock:
            return self._require_robot().send_cmd(command)

    def get_numeric_register(self, register: int) -> Numeric:
        """Read a FANUC numeric register and preserve integer values."""
        register = self._validate_register_index(register)
        with self._lock:
            _, response = self._require_robot().send_cmd(f"getreg:{register:03}")

        try:
            return int(response)
        except ValueError:
            value = float(response)
            if not math.isfinite(value):
                raise FanucpyTransportError(
                    "Controller returned a non-finite register value"
                )
            return value

    @staticmethod
    def validate_digital_output(
        output_type: str,
        output_number: int,
    ) -> Tuple[str, int]:
        """Return a canonical output type and controller-supported number."""
        output_type = str(output_type).strip().upper()
        if output_type == "DO":
            output_type = "DOUT"
        output_number = int(output_number)
        if output_type == "RDO":
            if not 1 <= output_number <= 9:
                raise ValueError("RDO number must be in the range 1..9")
        elif output_type == "DOUT":
            if not 1 <= output_number <= 99999:
                raise ValueError("DOUT number must be in the range 1..99999")
        else:
            raise ValueError("Output type must be RDO or DOUT")
        return output_type, output_number

    def get_digital_output(self, output_type: str, output_number: int) -> bool:
        """Read one RDO or DOUT value through a validated MAPPDK command."""
        output_type, output_number = self.validate_digital_output(
            output_type,
            output_number,
        )
        command = (
            f"getrdo:{output_number}"
            if output_type == "RDO"
            else f"getdout:{output_number:05d}"
        )
        with self._lock:
            _, response = self._require_robot().send_cmd(command)
        if response not in ("0", "1"):
            raise FanucpyTransportError(
                f"Controller returned invalid {output_type} value: {response}"
            )
        return response == "1"

    def set_digital_output(
        self,
        output_type: str,
        output_number: int,
        value: bool,
    ) -> Tuple[int, str]:
        """Write one RDO or DOUT value through a validated MAPPDK command."""
        if not isinstance(value, bool):
            raise TypeError("Digital output value must be boolean")
        output_type, output_number = self.validate_digital_output(
            output_type,
            output_number,
        )
        command_name = "setrdo" if output_type == "RDO" else "setdout"
        formatted_number = (
            str(output_number)
            if output_type == "RDO"
            else f"{output_number:05d}"
        )
        command = f"{command_name}:{formatted_number}:{str(value).lower()}"
        with self._lock:
            return self._require_robot().send_cmd(command)

    def set_gripper(self, value: bool) -> Tuple[int, str]:
        """Write the configured end-effector digital output."""
        if self.ee_do_type is None or self.ee_do_num is None:
            raise FanucpyTransportError("Gripper output is not configured")
        return self.set_digital_output(self.ee_do_type, self.ee_do_num, value)

    def get_instantaneous_power_w(self) -> float:
        """Read the controller power estimate and convert kW to watts."""
        try:
            with self._lock:
                _, response = self._require_robot().send_cmd("ins_pwr")
        except Exception as exc:
            if str(exc).strip().lower() == "wrong-command":
                raise FanucpyTransportError(
                    "the loaded MAPPDK server does not support the optional "
                    "ins_pwr command; install one matching, fully compiled "
                    "MAPPDK controller file set"
                ) from exc
            raise
        power_w = float(response) * 1000.0
        if not math.isfinite(power_w):
            raise FanucpyTransportError(
                "Controller returned a non-finite power value"
            )
        return power_w

    @classmethod
    def normalize_program_name(cls, program_name: str) -> str:
        """Return one canonical FANUC program name or reject it."""
        normalized = str(program_name).strip().upper()
        if not cls._PROGRAM_NAME.fullmatch(normalized):
            raise ValueError(
                "FANUC program name must contain 1..32 letters, digits, or "
                "underscores and must begin with a letter"
            )
        return normalized

    def call_program(self, program_name: str) -> Tuple[int, str]:
        """Dispatch a teach-pendant program after validating its name."""
        program_name = self.normalize_program_name(program_name)
        with self._lock:
            return self._require_robot().call_prog(program_name)

    def call_program_with_session_recycle(
        self,
        program_name: str,
        reconnect_timeout_sec: float,
        reconnect_retry_sec: float,
        state_probe_timeout_sec: float = 5.0,
    ) -> ProgramCallResult:
        """
        Run one TP program, then replace and verify the MAPPDK session.

        Some controller builds stop servicing commands on a socket after
        ``mappdkcall``. The transport lock remains held from the program call
        through recovery so state polling cannot issue a command on that stale
        session.
        """
        program_name = self.normalize_program_name(program_name)
        reconnect_timeout_sec = float(reconnect_timeout_sec)
        reconnect_retry_sec = float(reconnect_retry_sec)
        state_probe_timeout_sec = float(state_probe_timeout_sec)
        if reconnect_timeout_sec <= 0.0:
            raise ValueError("Program reconnect timeout must be greater than zero")
        if reconnect_retry_sec <= 0.0:
            raise ValueError("Program reconnect retry delay must be greater than zero")
        if state_probe_timeout_sec <= 0.0:
            raise ValueError("Program state-probe timeout must be greater than zero")

        with self._lock:
            code, message = self._require_robot().call_prog(program_name)
            code = int(code)
            message = str(message)
            if code != 0:
                return ProgramCallResult(
                    response_code=code,
                    response_message=message,
                    connection_ready=self._connected,
                    recovery_message="Program command failed; session not recycled",
                    recovered_state=None,
                )

            self._disconnect_unlocked()
            deadline = time.monotonic() + reconnect_timeout_sec
            last_error: Optional[Exception] = None

            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                time.sleep(min(reconnect_retry_sec, max(remaining, 0.0)))
                if time.monotonic() >= deadline:
                    break

                try:
                    self._connect_unlocked()
                    robot = self._require_robot()
                    comm_sock = getattr(robot, "comm_sock", None)
                    previous_timeout: Optional[float] = None
                    if comm_sock is not None and hasattr(comm_sock, "settimeout"):
                        if hasattr(comm_sock, "gettimeout"):
                            previous_timeout = comm_sock.gettimeout()
                        comm_sock.settimeout(
                            min(state_probe_timeout_sec, reconnect_timeout_sec)
                        )
                    try:
                        snapshot = self._read_state_unlocked()
                    finally:
                        if comm_sock is not None and hasattr(
                            comm_sock,
                            "settimeout",
                        ):
                            comm_sock.settimeout(previous_timeout)
                    return ProgramCallResult(
                        response_code=code,
                        response_message=message,
                        connection_ready=True,
                        recovery_message=(
                            "MAPPDK socket session recycled and state verified"
                        ),
                        recovered_state=snapshot,
                    )
                except Exception as exc:
                    last_error = exc
                    self._disconnect_unlocked()

            detail = str(last_error) if last_error is not None else "timed out"
            return ProgramCallResult(
                response_code=code,
                response_message=message,
                connection_ready=False,
                recovery_message=(
                    "TP program reported success, but MAPPDK session recovery "
                    f"failed: {detail}; do not automatically repeat the program"
                ),
                recovered_state=None,
            )
