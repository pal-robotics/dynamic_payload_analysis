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


from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration





def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dynamic_payload_analysis_ros'), 'config/rviz/', 'talos_dyn_analysis.rviz'])


    robot_state_publisher = include_launch_py_description(
        "talos_description", ["launch", "robot_state_publisher.launch.py"],
    )

    analysis_node = include_launch_py_description(
        "dynamic_payload_analysis_ros", ["launch", "dyn_payload_analysis.launch.py"],
    )


    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    ld = LaunchDescription()


    ld.add_action(robot_state_publisher)
    ld.add_action(start_rviz_cmd)
    ld.add_action(analysis_node)

    return ld
