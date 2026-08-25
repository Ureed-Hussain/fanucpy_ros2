# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

import importlib
import sys
from types import ModuleType

import pytest


class FakeUpstreamRobot:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.commands = []
        self.register_response = "0"

    def send_cmd(self, command, continue_on_error=False):
        self.commands.append((command, continue_on_error))
        return 0, self.register_response


@pytest.fixture
def robot_module(monkeypatch):
    """Load the compatibility module against a fake upstream dependency."""
    module_name = "fanucpy_ros2_driver.robot"
    previous_module = sys.modules.pop(module_name, None)
    fake_fanucpy = ModuleType("fanucpy")
    fake_fanucpy.Robot = FakeUpstreamRobot
    monkeypatch.setitem(sys.modules, "fanucpy", fake_fanucpy)
    module = importlib.import_module(module_name)
    yield module
    sys.modules.pop(module_name, None)
    if previous_module is not None:
        sys.modules[module_name] = previous_module


def test_set_reg_formats_integer_and_real_commands(robot_module):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    robot.set_reg(101, 42)
    robot.set_reg(102, 12.5, continue_on_error=True)
    assert robot.commands == [
        ("setregint:101:42", False),
        ("setregflt:102:12.5", True),
    ]


@pytest.mark.parametrize("register", [0, 1000])
def test_set_reg_rejects_out_of_range_register(robot_module, register):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    with pytest.raises(ValueError, match="range 1..999"):
        robot.set_reg(register, 1)


@pytest.mark.parametrize("value", [True, float("inf"), float("nan")])
def test_set_reg_rejects_invalid_values(robot_module, value):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    with pytest.raises((TypeError, ValueError)):
        robot.set_reg(101, value)


def test_get_reg_preserves_integer_and_float_responses(robot_module):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    robot.register_response = "42"
    assert robot.get_reg(101) == 42
    robot.register_response = "12.5"
    assert robot.get_reg(102) == 12.5


def test_set_pr_formats_six_axis_cartesian_command(robot_module):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    robot.set_pr(
        12,
        "xyz",
        [123.5, -2, 0, 180, -90.25, 1],
        continue_on_error=True,
    )
    assert robot.commands == [
        (
            "setpr:6:012:xyz:+000123.500000:-000002.000000:"
            "+000000.000000:+000180.000000:-000090.250000:"
            "+000001.000000",
            True,
        )
    ]


@pytest.mark.parametrize(
    ("pr_type", "values", "exception"),
    [
        ("invalid", [0, 0, 0, 0, 0, 0], ValueError),
        ("joint", [0, 0, 0], ValueError),
        ("joint", [0, 0, 0, 0, 0, True], TypeError),
        ("joint", [0, 0, 0, 0, 0, float("inf")], ValueError),
    ],
)
def test_set_pr_rejects_invalid_inputs(
    robot_module,
    pr_type,
    values,
    exception,
):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    with pytest.raises(exception):
        robot.set_pr(12, pr_type, values)


def test_get_pr_parses_six_axis_response(robot_module):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    robot.register_response = "1.0;2;3.5;4;5;6"
    assert robot.get_pr(12) == [1.0, 2.0, 3.5, 4.0, 5.0, 6.0]
    assert robot.commands == [("getpr:012", False)]


def test_get_pr_rejects_malformed_response(robot_module):
    robot = robot_module.Robot(robot_model="Fanuc", host="192.168.0.177")
    robot.register_response = "1;2;3"
    with pytest.raises(ValueError, match="invalid position register"):
        robot.get_pr(12)
