# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

import argparse

import pytest

from fanucpy_ros2_examples.controller_tools import (
    boolean_value,
    build_parser,
    format_power_result,
)


@pytest.mark.parametrize(("text", "expected"), [("true", True), ("off", False)])
def test_boolean_value(text, expected):
    assert boolean_value(text) is expected


def test_invalid_boolean_value_is_rejected():
    with pytest.raises(argparse.ArgumentTypeError):
        boolean_value("maybe")


def test_valid_power_result_includes_measurement():
    assert format_power_result(True, 1250.0, "ok") == "power=1250.000 W; ok"


def test_failed_power_result_does_not_claim_zero_watts():
    message = "Power read failed: wrong-command"
    assert format_power_result(False, 0.0, message) == message


def test_parser_builds_read_register_command():
    command = build_parser().parse_args(["get-register", "101"])
    assert command.command == "get-register"
    assert command.register_number == 101
    assert command.driver_namespace == "/fanuc"


def test_parser_builds_allowlisted_program_command():
    command = build_parser().parse_args(["run-program", "HOME_P"])
    assert command.command == "run-program"
    assert command.program_name == "HOME_P"
