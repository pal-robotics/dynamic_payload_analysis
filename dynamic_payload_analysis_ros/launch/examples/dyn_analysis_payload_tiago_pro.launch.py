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
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    wrist_model_right: DeclareLaunchArgument = TiagoProArgs.wrist_model_right
    wrist_model_left: DeclareLaunchArgument = TiagoProArgs.wrist_model_left
    camera_model: DeclareLaunchArgument = TiagoProArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoProArgs.laser_model
    tool_changer_right: DeclareLaunchArgument = TiagoProArgs.tool_changer_right
    tool_changer_left: DeclareLaunchArgument = TiagoProArgs.tool_changer_left
    torque_estimation: DeclareLaunchArgument = TiagoProArgs.torque_estimation
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    
    advanced_mode: DeclareLaunchArgument = DeclareLaunchArgument(
            name='advanced_mode',
            default_value='false',
            description='If true, it enables to add payload in every links of kinematic trees')

    resolution_ik: DeclareLaunchArgument = DeclareLaunchArgument(
            name='resolution_ik',
            default_value='0.20',
            description='Resolution of IK solver')

    workspace_range: DeclareLaunchArgument = DeclareLaunchArgument(
            name='workspace_range',
            default_value='2.0',
            description='Range of IK solver')

def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

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
    

    launch_description.add_action(analysis_node)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='tiago_pro_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={'arm_type_right': launch_args.arm_type_right,
                          'arm_type_left': launch_args.arm_type_left,
                          "end_effector_right": launch_args.end_effector_right,
                          "end_effector_left": launch_args.end_effector_left,
                          "ft_sensor_right": launch_args.ft_sensor_right,
                          "ft_sensor_left": launch_args.ft_sensor_left,
                          "wrist_model_right": launch_args.wrist_model_right,
                          "wrist_model_left": launch_args.wrist_model_left,
                          "laser_model": launch_args.laser_model,
                          "camera_model": launch_args.camera_model,
                          "base_type": launch_args.base_type,
                          'tool_changer_right': launch_args.tool_changer_right,
                          'tool_changer_left': launch_args.tool_changer_left,
                          'torque_estimation': launch_args.torque_estimation,
                          "namespace": launch_args.namespace,
                          "use_sim_time": launch_args.use_sim_time
                          })

    launch_description.add_action(robot_state_publisher)

    

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dynamic_payload_analysis_ros'), 'config/rviz/', 'tiago_pro_dyn_analysis.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')
                     }])
    launch_description.add_action(rviz)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
