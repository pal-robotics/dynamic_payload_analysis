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
    advanced_mode = DeclareLaunchArgument(
        name='advanced_mode',
        default_value='false',
        description='If true, it enables to add payload in every links of kinematic trees')

    resolution_ik = DeclareLaunchArgument(
        name='resolution_ik',
        default_value='0.20',
        description='Resolution of IK solver')

    workspace_range = DeclareLaunchArgument(
        name='workspace_range',
        default_value='2.0',
        description='Range of IK solver')


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dynamic_payload_analysis_ros'), 'config/rviz/', 'talos_dyn_analysis.rviz'])


    robot_state_publisher = include_launch_py_description(
        "talos_description", ["launch", "robot_state_publisher.launch.py"],
    )

    analysis_node = Node(
        package="dynamic_payload_analysis_ros",
        executable='node_rviz_visualization_menu',
        name='dynamic_analysis_node',
        output='screen',
        parameters=[{'advanced_mode': LaunchConfiguration('advanced_mode'),
                     'resolution_ik': LaunchConfiguration('resolution_ik'),
                     'workspace_range': LaunchConfiguration('workspace_range'),
                     }]
        )


    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(advanced_mode)
    ld.add_action(resolution_ik)
    ld.add_action(workspace_range)

    ld.add_action(robot_state_publisher)
    ld.add_action(start_rviz_cmd)
    ld.add_action(analysis_node)

    return ld
