# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

"""Source-controlled fanucpy compatibility class used by the ROS driver."""

import math
import numbers
from typing import Literal, Union

from fanucpy import Robot as _FanucpyRobot


Numeric = Union[int, float]


class Robot(_FanucpyRobot):
    """
    Extend upstream ``fanucpy.Robot`` with validated numeric registers.

    This keeps the project's former ``site-packages/fanucpy/robot.py`` change
    in the ROS workspace. All other connection, state, motion, I/O, gripper,
    power, and TP-program behavior remains implemented by upstream fanucpy.
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
