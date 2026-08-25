# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Source-controlled fanucpy compatibility class used by the ROS driver."""

import math
import numbers
from typing import Literal, Sequence, Union

from fanucpy import Robot as _FanucpyRobot


Numeric = Union[int, float]
PositionRegisterType = Literal["xyz", "joint"]


class Robot(_FanucpyRobot):
    """
    Extend upstream ``fanucpy.Robot`` with validated numeric registers.

    This keeps the project's former ``site-packages/fanucpy/robot.py`` change
    in the ROS workspace. It also preserves the six-axis position-register
    methods from the supplied project bundle, with their type and command bugs
    corrected. All other connection, state, motion, I/O, gripper, power, and
    TP-program behavior remains implemented by upstream fanucpy.
    """

    _INTEGER_REGISTER_MIN = -(2**31)
    _INTEGER_REGISTER_MAX = 2**31 - 1

    @staticmethod
    def _validate_register_number(register: int) -> int:
        if isinstance(register, bool) or not isinstance(
            register,
            numbers.Integral,
        ):
            raise TypeError("Numeric register number must be an integer")
        register_number = int(register)
        if not 1 <= register_number <= 999:
            raise ValueError("Numeric register number must be in the range 1..999")
        return register_number

    def set_reg(
        self,
        reg_num: int,
        val: Numeric,
        continue_on_error: bool = False,
    ) -> tuple[Literal[0, 1], str]:
        """Set R[] while preserving integer and real scalar types."""
        register = self._validate_register_number(reg_num)
        if isinstance(val, bool):
            raise TypeError("Boolean values are not numeric register values")

        if isinstance(val, numbers.Integral):
            integer_value = int(val)
            if not (
                self._INTEGER_REGISTER_MIN
                <= integer_value
                <= self._INTEGER_REGISTER_MAX
            ):
                raise ValueError("Integer register value exceeds signed 32-bit range")
            command = f"setregint:{register:03}:{integer_value}"
        elif isinstance(val, numbers.Real):
            real_value = float(val)
            if not math.isfinite(real_value):
                raise ValueError("Numeric register value must be finite")
            command = f"setregflt:{register:03}:{real_value}"
        else:
            raise TypeError(f"Unsupported numeric register value: {type(val)!r}")

        return self.send_cmd(
            command,
            continue_on_error=continue_on_error,
        )

    def get_reg(
        self,
        reg_num: int,
        continue_on_error: bool = False,
    ) -> Numeric:
        """Read R[] and preserve an integer response when possible."""
        register = self._validate_register_number(reg_num)
        _, response = self.send_cmd(
            f"getreg:{register:03}",
            continue_on_error=continue_on_error,
        )
        try:
            return int(response)
        except ValueError:
            value = float(response)
            if not math.isfinite(value):
                raise ValueError(
                    "Controller returned a non-finite numeric register value"
                )
            return value

    @staticmethod
    def _validate_position_register_number(register: int) -> int:
        if isinstance(register, bool) or not isinstance(
            register,
            numbers.Integral,
        ):
            raise TypeError("Position register number must be an integer")
        register_number = int(register)
        if not 1 <= register_number <= 999:
            raise ValueError("Position register number must be in the range 1..999")
        return register_number

    def set_pr(
        self,
        pr_num: int,
        pr_type: PositionRegisterType,
        val: Sequence[Numeric],
        continue_on_error: bool = False,
    ) -> tuple[Literal[0, 1], str]:
        """Set a six-axis Cartesian or joint position register."""
        register = self._validate_position_register_number(pr_num)
        if pr_type not in ("xyz", "joint"):
            raise ValueError("Position register type must be 'xyz' or 'joint'")
        if isinstance(val, (str, bytes)):
            raise TypeError("Position register values must be a numeric sequence")

        values = list(val)
        if len(values) != 6:
            raise ValueError("Position register requires exactly six values")

        command = f"setpr:6:{register:03}:{pr_type}"
        for coordinate in values:
            if isinstance(coordinate, bool) or not isinstance(
                coordinate,
                numbers.Real,
            ):
                raise TypeError("Position register values must be real numbers")
            numeric_coordinate = float(coordinate)
            if not math.isfinite(numeric_coordinate):
                raise ValueError("Position register values must be finite")
            command += f":{numeric_coordinate:+014.6f}"

        return self.send_cmd(
            command,
            continue_on_error=continue_on_error,
        )

    def get_pr(
        self,
        pr_num: int,
        continue_on_error: bool = False,
    ) -> list[float]:
        """Read a six-axis position register response."""
        register = self._validate_position_register_number(pr_num)
        _, response = self.send_cmd(
            f"getpr:{register:03}",
            continue_on_error=continue_on_error,
        )
        values = [float(value) for value in response.split(";")]
        if len(values) != 6:
            raise ValueError("Controller returned an invalid position register")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Controller returned non-finite position register values")
        return values
