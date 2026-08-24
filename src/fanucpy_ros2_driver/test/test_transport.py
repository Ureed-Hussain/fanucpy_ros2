# Copyright 2026 ureed
# SPDX-License-Identifier: Apache-2.0

import pytest

from fanucpy_ros2_driver.transport import (
    FanucpyTransport,
    FanucpyTransportError,
)


class FakeRobot:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.commands = []
        self.disconnected = False

    def connect(self):
        return 0, "connected"

    def disconnect(self):
        self.disconnected = True

    def get_curjpos(self):
        return [0, 10, 20, 30, 40, 50]

    def get_curpos(self):
        return [100, 200, 300, 0, 90, 180]

    def send_cmd(self, command):
        self.commands.append(command)
        if command.startswith("getreg"):
            return 0, "12.5"
        if command.startswith(("getrdo", "getdout")):
            return 0, "1"
        if command == "ins_pwr":
            return 0, "1.25"
        return 0, "success"

    def call_prog(self, name):
        self.commands.append(f"program:{name}")
        return 0, "success"

    def move(self, move_type, values, **kwargs):
        self.commands.append((move_type, values, kwargs))
        return 0, "move complete"


def make_transport():
    return FanucpyTransport(
        robot_model="Fanuc",
        host="192.0.2.10",
        robot_factory=FakeRobot,
    )


class RecordingRobotFactory:
    def __init__(self):
        self.instances = []

    def __call__(self, **kwargs):
        robot = FakeRobot(**kwargs)
        self.instances.append(robot)
        return robot


def test_connect_and_read_state():
    transport = make_transport()
    assert transport.connect() == (0, "connected")
    state = transport.read_state()
    assert state.joints_deg == (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
    assert state.cartesian_mm_deg == (100.0, 200.0, 300.0, 0.0, 90.0, 180.0)


def test_register_extension_formats_integer_and_float_commands():
    transport = make_transport()
    transport.connect()
    transport.set_numeric_register(101, 42)
    transport.set_numeric_register(102, 12.5)
    assert transport._robot.commands == [
        "setregint:101:42",
        "setregflt:102:12.5",
    ]


def test_get_numeric_register_returns_float():
    transport = make_transport()
    transport.connect()
    assert transport.get_numeric_register(101) == 12.5


def test_cartesian_jog_is_relative_to_live_pose():
    transport = make_transport()
    transport.connect()
    result = transport.jog_cartesian(
        (1.0, -2.0, 3.0, 0.5, 0.0, -0.5),
        velocity_mm_s=25,
        acceleration_percent=20,
    )
    assert result.target_mm_deg == (101.0, 198.0, 303.0, 0.5, 90.0, 179.5)
    assert result.response_code == 0
    assert transport._robot.commands[-1] == (
        "pose",
        [101.0, 198.0, 303.0, 0.5, 90.0, 179.5],
        {
            "velocity": 25,
            "acceleration": 20,
            "cnt_val": 0,
            "linear": True,
        },
    )


def test_joint_waypoint_uses_exact_stop_and_returns_measured_joints():
    transport = make_transport()
    transport.connect()
    result = transport.move_joint(
        (1, 2, 3, 4, 5, 6),
        velocity_percent=10,
        acceleration_percent=20,
    )
    assert result.target_deg == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert result.actual_deg == (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
    assert transport._robot.commands[-1] == (
        "joint",
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        {
            "velocity": 10,
            "acceleration": 20,
            "cnt_val": 0,
            "linear": False,
        },
    )


@pytest.mark.parametrize("velocity", [0, 101])
def test_joint_waypoint_velocity_is_bounded(velocity):
    transport = make_transport()
    transport.connect()
    with pytest.raises(ValueError):
        transport.move_joint(
            (1, 2, 3, 4, 5, 6),
            velocity_percent=velocity,
            acceleration_percent=20,
        )


@pytest.mark.parametrize("register", [0, 1000])
def test_invalid_register_index_is_rejected(register):
    transport = make_transport()
    transport.connect()
    with pytest.raises(ValueError):
        transport.set_numeric_register(register, 1)


def test_boolean_register_value_is_rejected():
    transport = make_transport()
    transport.connect()
    with pytest.raises(TypeError):
        transport.set_numeric_register(101, True)


@pytest.mark.parametrize("value", [-(2**31) - 1, 2**31])
def test_integer_register_value_is_signed_32_bit(value):
    transport = make_transport()
    transport.connect()
    with pytest.raises(ValueError):
        transport.set_numeric_register(101, value)


def test_rdo_and_dout_commands_are_formatted_for_mappdk():
    transport = make_transport()
    transport.connect()
    assert transport.get_digital_output("rdo", 7) is True
    assert transport.get_digital_output("DO", 42) is True
    transport.set_digital_output("RDO", 7, False)
    transport.set_digital_output("DOUT", 42, True)
    assert transport._robot.commands[-4:] == [
        "getrdo:7",
        "getdout:00042",
        "setrdo:7:false",
        "setdout:00042:true",
    ]


@pytest.mark.parametrize(
    ("output_type", "number"),
    [("RDO", 0), ("RDO", 10), ("DOUT", 0), ("DOUT", 100000), ("IO", 1)],
)
def test_invalid_digital_output_is_rejected(output_type, number):
    transport = make_transport()
    transport.connect()
    with pytest.raises(ValueError):
        transport.get_digital_output(output_type, number)


def test_gripper_uses_configured_output():
    transport = make_transport()
    transport.connect()
    transport.set_gripper(True)
    assert transport._robot.commands[-1] == "setrdo:7:true"


def test_power_is_converted_from_kw_to_watts():
    transport = make_transport()
    transport.connect()
    assert transport.get_instantaneous_power_w() == 1250.0


def test_unsupported_power_command_has_controller_version_guidance():
    class PowerUnsupportedRobot(FakeRobot):
        def send_cmd(self, command):
            if command == "ins_pwr":
                raise RuntimeError("wrong-command")
            return super().send_cmd(command)

    transport = FanucpyTransport(
        robot_model="Fanuc",
        host="192.0.2.10",
        robot_factory=PowerUnsupportedRobot,
    )
    transport.connect()
    with pytest.raises(FanucpyTransportError, match="does not support"):
        transport.get_instantaneous_power_w()


def test_program_name_is_validated():
    transport = make_transport()
    transport.connect()
    transport.call_program(" home_p ")
    assert transport._robot.commands[-1] == "program:HOME_P"
    with pytest.raises(ValueError):
        transport.call_program("HOME:P")
    with pytest.raises(ValueError):
        transport.call_program("A" * 33)


def test_program_session_is_replaced_and_state_verified_atomically():
    factory = RecordingRobotFactory()
    transport = FanucpyTransport(
        robot_model="Fanuc",
        host="192.0.2.10",
        robot_factory=factory,
    )
    transport.connect()

    result = transport.call_program_with_session_recycle(
        "HOME_P",
        reconnect_timeout_sec=0.1,
        reconnect_retry_sec=0.001,
        state_probe_timeout_sec=0.05,
    )

    assert result.response_code == 0
    assert result.connection_ready is True
    assert result.recovered_state is not None
    assert len(factory.instances) == 2
    assert factory.instances[0].commands == ["program:HOME_P"]
    assert factory.instances[0].disconnected is True
    assert transport.connected is True


@pytest.mark.parametrize("timeout", [0.0, -1.0])
def test_program_session_recycle_timeout_must_be_positive(timeout):
    transport = make_transport()
    transport.connect()
    with pytest.raises(ValueError):
        transport.call_program_with_session_recycle(
            "HOME_P",
            reconnect_timeout_sec=timeout,
            reconnect_retry_sec=0.01,
        )
