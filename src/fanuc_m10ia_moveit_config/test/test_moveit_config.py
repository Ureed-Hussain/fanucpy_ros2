# Copyright 2026 ureed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Static consistency tests which never connect to a robot."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config"
JOINTS = [f"joint_{number}" for number in range(1, 7)]


def _yaml(name: str):
    with (CONFIG / name).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_required_configuration_files_exist():
    required = {
        "fanuc_m10ia.urdf.xacro",
        "fanuc_m10ia.srdf",
        "initial_positions.yaml",
        "joint_limits.yaml",
        "kinematics.yaml",
        "moveit_controllers.yaml",
        "moveit.rviz",
        "ompl_planning.yaml",
        "ros2_controllers.yaml",
        "sensors_3d.yaml",
    }
    assert required <= {path.name for path in CONFIG.iterdir()}


def test_srdf_chain_and_joint_state_are_consistent():
    root = ET.parse(CONFIG / "fanuc_m10ia.srdf").getroot()
    group = root.find("./group[@name='manipulator']")
    assert group is not None
    chain = group.find("chain")
    assert chain is not None
    assert chain.attrib == {"base_link": "base_link", "tip_link": "tool0"}

    state_joints = root.findall(
        "./group_state[@name='all_zeros']/joint"
    )
    assert [joint.attrib["name"] for joint in state_joints] == JOINTS


def test_moveit_controller_matches_real_driver_action():
    controllers = _yaml("moveit_controllers.yaml")
    manager = controllers["moveit_simple_controller_manager"]
    assert manager["controller_names"] == ["fanuc_arm_controller"]
    controller = manager["fanuc_arm_controller"]
    assert controller["action_ns"] == "follow_joint_trajectory"
    assert controller["joints"] == JOINTS
    assert (
        controllers["trajectory_execution"]["execution_duration_monitoring"]
        is False
    )


def test_mock_controller_uses_same_name_and_joints():
    controllers = _yaml("ros2_controllers.yaml")
    declared = controllers["controller_manager"]["ros__parameters"]
    assert "fanuc_arm_controller" in declared
    assert (
        controllers["fanuc_arm_controller"]["ros__parameters"]["joints"]
        == JOINTS
    )


def test_joint_limits_cover_exactly_six_driver_joints():
    limits = _yaml("joint_limits.yaml")["joint_limits"]
    assert list(limits) == JOINTS
    assert all(limits[joint]["max_velocity"] > 0.0 for joint in JOINTS)


def test_description_uses_licensed_external_m10ia_model():
    xacro = (CONFIG / "fanuc_m10ia.urdf.xacro").read_text(
        encoding="utf-8"
    )
    assert "moveit_resources_fanuc_description" in xacro
    assert "mock_components/GenericSystem" in xacro
