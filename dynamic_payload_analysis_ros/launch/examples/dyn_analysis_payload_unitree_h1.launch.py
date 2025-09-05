# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.include_utils import include_launch_py_description



h1_xacro_filepath = os.path.join(
    get_package_share_directory("dynamic_payload_analysis_core"),
    "examples",
    "urdf",
    "h1_description",
    "urdf",
    "h1_with_hand.urdf",
)

robot_description = xacro.process_file(
    h1_xacro_filepath
).toprettyxml(indent="  ")


def generate_launch_description():
    rviz_file = os.path.join(
        get_package_share_directory("dynamic_payload_analysis_ros"),
        "config",
        "rviz",
        "h1_dyn_analysis.rviz",
    )

    analysis_node = include_launch_py_description(
        "dynamic_payload_analysis_ros", ["launch", "dyn_payload_analysis.launch.py"],
    )

    return LaunchDescription(
        [
            analysis_node,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            ),
        ]
    )
