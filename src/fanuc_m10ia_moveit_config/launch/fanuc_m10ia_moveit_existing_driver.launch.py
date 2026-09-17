# Copyright 2026 Muhammad Ureed Hussain
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

"""Open real M-10iA MoveIt using an existing driver; never start a driver."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Fix real mode and driver reuse for the conversational example launcher."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("fanuc_m10ia_moveit_config"), "launch",
                "fanuc_m10ia_moveit.launch.py",
            ])),
            launch_arguments={
                "mode": "real", "start_driver": "false", "use_rviz": "true",
            }.items(),
        ),
    ])
