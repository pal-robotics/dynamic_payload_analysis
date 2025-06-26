#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import WrenchStamped, Point
from sensor_msgs.msg import JointState
from dynamic_payload_analysis_core.core import TorqueCalculator
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_payload_analysis_ros.menu_visual import MenuPayload




class RobotDescriptionSubscriber(Node):
    def __init__(self):
        super().__init__('node_robot_description_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos_profile=rclpy.qos.QoSProfile( durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth = 1)
        )

        # menu for selecting frames to apply payload
        self.menu = MenuPayload(self)

        # Publisher for external force
        self.publisher_force = self.create_publisher(MarkerArray, '/external_forces', 10)

        # Publisher for RViz visualization of torques
        self.publisher_rviz_torque = self.create_publisher(MarkerArray, '/torque_visualization', 10)

        # Pusblisher for point cloud workspace area
        self.publisher_workspace_area = self.create_publisher(Marker, '/workspace_area', 10)

        # Publisher for point names in the workspace area
        self.publisher_workspace_area_names = self.create_publisher(MarkerArray, '/workspace_area_names', 10)

        # subscription to joint states
        self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # publisher for the joint states
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        
        self.robot = None

        # frame where the external force is applied
        self.frame_id = None

        self.external_force = None

        # variable to store the currente selected configuration from the workspace menu
        self.valid_configurations = None

        # variable to store if there is a selected configuration from the workspace menu to visualize
        self.selected_configuration = None

        # timer to compute the valid workspace area 
        self.timer_workspace_calculation = self.create_timer(3, self.workspace_calculation)
        # timer to publish the selected configuration in joint states
        self.timer_publish_configuration = self.create_timer(2, self.publish_selected_configuration)
        # timer to publish the external forces as arrows in RViz
        self.timer_publish_force = self.create_timer(1, self.publish_payload_force)

        self.get_logger().info('Robot description subscriber node started')


    def publish_payload_force(self):
        """
        Publish the gravity force on the frame with id `id_force`.
        """
        external_force_array = MarkerArray()
        
        for frame in self.menu.get_item_state():

            id_force = self.robot.get_parent_joint_id(frame["name"])
            joint_position = self.robot.get_joint_placement(id_force)
            arrow_force = Marker()

            arrow_force.header.frame_id = "base_link" 
            arrow_force.header.stamp = self.get_clock().now().to_msg()
            arrow_force.ns = "external_force"
            arrow_force.id = id_force
            arrow_force.type = Marker.ARROW

            # add the arrow if the frame is checked or delete it if not
            if frame["checked"]:
                arrow_force.action = Marker.ADD
            else:
                arrow_force.action = Marker.DELETE

            arrow_force.scale.x = 0.20   # Length of the arrow
            arrow_force.scale.y = 0.05   # Width of the arrow
            arrow_force.scale.z = 0.05   # Height of the arrow
            arrow_force.color.a = 1.0  # Alpha
            arrow_force.color.r = 0.0
            arrow_force.color.g = 0.0  # Green
            arrow_force.color.b = 1.0

            # Set the position of the arrow at the joint placement
            arrow_force.pose.position.x = joint_position["x"]
            arrow_force.pose.position.y = joint_position["y"]
            arrow_force.pose.position.z = joint_position["z"]
            # Set the direction of the arrow downwards
            arrow_force.pose.orientation.x = 0.0
            arrow_force.pose.orientation.y = 0.7071
            arrow_force.pose.orientation.z = 0.0
            arrow_force.pose.orientation.w = 0.7071
            
            external_force_array.markers.append(arrow_force)
        
        self.publisher_force.publish(external_force_array)



    def workspace_calculation(self):
        """
        Timer to compute the valid workspace area.
        """
        if self.menu.get_workspace_state():
            self.valid_configurations = self.robot.get_valid_workspace(2, 0.3, "gripper_left_finger_joint", self.external_force)

            # publish the workspace area
            self.publish_workspace_area(self.valid_configurations)
            # set the workspace state to False to stop the computation
            self.menu.insert_dropdown_configuration(self.valid_configurations)
            
            # reset the workspace state to False to stop the computation
            self.menu.set_workspace_state(False)



    def publish_selected_configuration(self):
        """
        Timer to publish the selected configuration.
        This will publish the joint states of the selected configuration in the menu.
        """
        self.selected_configuration = self.menu.get_selected_configuration()
        
        if self.selected_configuration is not None:
            configs = self.robot.get_position_for_joint_states(self.valid_configurations[self.selected_configuration]["config"])
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            
            joint_state.name = [joint["joint_name"] for joint in configs]
            joint_state.position = [joint["q"] for joint in configs]
            #joint_state.position =
            self.publisher_joint_states.publish(joint_state)

        

    def robot_description_callback(self, msg):
        self.get_logger().info('Received robot description')

        self.robot = TorqueCalculator(robot_description = msg.data)
        self.robot.print_active_joint()
        self.robot.print_frames()
        
        # Add the frame to the menu for payload selection
        for frame in self.robot.get_active_frames():
            self.menu.insert_frame(frame)

        # self.robot.print_configuration()


    def joint_states_callback(self, msg):
        if self.robot is not None:
            # if you are not using the calculated configuration from workspace, you can use the joint states to compute the torques because you don't have already the computed torques
            if self.selected_configuration is None:
                positions = list(msg.position)
                name_position = list(msg.name)
                v = msg.velocity if msg.velocity else self.robot.get_zero_velocity()
                
                # set the positions based on the joint states
                q = self.robot.set_position(positions, name_position)
                a = self.robot.get_zero_acceleration()
                        
                # create the array with only the checked frames (with external force applied)
                self.checked_frames = np.array([check_frame["name"] for check_frame in self.menu.get_item_state() if check_frame['checked']])
                
                # if there are no checked frames, set the external force to None
                if len(self.checked_frames) != 0:
                    # create the array with the masses of the checked frames
                    self.masses = np.array([check_frame["payload"] for check_frame in self.menu.get_item_state() if check_frame['checked']])
                    # create the external force with the masses and the checked frames
                    self.external_force = self.robot.create_ext_force(masses=self.masses, frame_name=self.checked_frames, q=q)
                else:
                    self.external_force = None

                # compute the inverse dynamics
                tau = self.robot.compute_inverse_dynamics(q, v, a, extForce=self.external_force)

                # check the effort limits
                status_efforts = self.robot.check_effort_limits(tau)
                # print the torques
                self.robot.print_torques(tau)

                # get the positions of the joints where the torques are applied based on 
                joints_position = self.robot.get_joints_placements(q)

                # Publish the torques in RViz
                self.publish_label_torques(tau, status_torques=status_efforts ,joints_position=joints_position)

            else:
                # if you are using the calculated configuration from workspace, you can use the valid configurations to visualize the torques labels
                # Publish the torque labels for the selected configuration
                
                # get the positions of the joints where the torques are applied based on 
                joints_position = self.robot.get_joints_placements(self.valid_configurations[self.selected_configuration]["config"])

                # get the torques status (if the torques are within the limits)
                status_efforts = self.robot.check_effort_limits(self.valid_configurations[self.selected_configuration]["tau"])

                self.publish_label_torques(self.valid_configurations[self.selected_configuration]["tau"], status_torques=status_efforts ,joints_position=joints_position)
            



    def publish_label_torques(self, torque: np.ndarray, status_torques : np.ndarray, joints_position: np.ndarray):
        """
        Publish the torque with labels on the robot in RViz.

        Args:
            torque (np.ndarray): The torque to be published
            status_torques (np.ndarray): The status of the torques, True if the torque is within the limits, False otherwise
            joints_position (np.ndarray): The positions of the joints where the torques are applied
        """
        marker_array = MarkerArray()
        for i, (t, joint) in enumerate(zip(torque, joints_position)):
            # remove the gripper joints from the visualization TODO: make it more general (with MIMIC joints)
            if "gripper" not in joint['name']:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "torque_visualization"
                marker.id = i
                marker.type = Marker.TEXT_VIEW_FACING
                # Set the text based on the joint type
                if joint['type'] == 'JointModelPZ':
                    marker.text = f"{joint['name']}: {t:.2f} N"
                else:
                    marker.text = f"{joint['name']}: {t:.2f} Nm"

                marker.action = Marker.ADD
                marker.pose.position.x = joint['x']
                marker.pose.position.y = joint['y']
                marker.pose.position.z = joint['z']
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.a = 1.0  # Alpha
                if status_torques[i]:
                    marker.color.r = 0.0  # Red
                    marker.color.g = 1.0  # Green
                    marker.color.b = 0.0  # Blue
                else:
                    marker.color.r = 1.0  # Red
                    marker.color.g = 0.0  # Green
                    marker.color.b = 0.0  # Blue
                marker_array.markers.append(marker)
        
        self.publisher_rviz_torque.publish(marker_array)
        



    def publish_workspace_area(self, valid_configs: np.ndarray ):
        """
        Publish the workspace area in RViz using points.

        Args:
            valid_configs (np.ndarray): Current valid configurations of the robot.
        """
        # Create a MarkerArray to visualize the number of configuration of a specific point in the workspace
        marker_point_names = MarkerArray()

        # Create a Marker for the workspace area using points
        marker_points = Marker()
        marker_points.header.frame_id = "base_link"
        marker_points.header.stamp = self.get_clock().now().to_msg()
        marker_points.ns = "workspace_area"
        marker_points.id = 0
        marker_points.type = Marker.SPHERE_LIST
        marker_points.action = Marker.ADD
        marker_points.scale.x = 0.03  # Size of the spheres
        marker_points.scale.y = 0.03
        marker_points.scale.z = 0.03
        
        
        points_array = []
        points_colors = []

        # Iterate through the valid configurations and create markers
        for i, valid_config in enumerate(valid_configs):
            
            # create the label for the point
            marker_point_name = Marker()
            marker_point_name.header.frame_id = "base_link"
            marker_point_name.header.stamp = self.get_clock().now().to_msg()
            marker_point_name.ns = "workspace_area"
            marker_point_name.id = i + 1 
            marker_point_name.type = Marker.TEXT_VIEW_FACING
            marker_point_name.text = f"Config {i}"
            marker_point_name.action = Marker.ADD
            marker_point_name.pose.position.x = valid_config["end_effector_pos"][0]
            marker_point_name.pose.position.y = valid_config["end_effector_pos"][1]
            marker_point_name.pose.position.z = valid_config["end_effector_pos"][2] + 0.05  # Offset to avoid overlap with the sphere
            marker_point_name.pose.orientation.w = 1.0
            marker_point_name.scale.x = 0.02
            marker_point_name.scale.y = 0.02
            marker_point_name.scale.z = 0.02
            marker_point_name.color.a = 1.0  # Alpha
            marker_point_name.color.r = 1.0  # Red
            marker_point_name.color.g = 1.0  # Green
            marker_point_name.color.b = 1.0  # Blue

            # create the point to visualize the possible end effector position
            point = Point()

            point.x = valid_config["end_effector_pos"][0]
            point.y = valid_config["end_effector_pos"][1]
            point.z = valid_config["end_effector_pos"][2]

            # Add color based on the torque
            color = ColorRGBA()
            normalized_torque = self.robot.get_normalized_unified_torque(valid_config["tau"], "left")

            color.r = normalized_torque  # More red as torque approaches 1
            color.g = 1.0 - normalized_torque  # More green as torque approaches 0
            color.b = 0.0  # No blue component
            color.a = 1.0  # Alpha
            
            points_colors.append(color)
            points_array.append(point)

            # Add the marker for the point
            marker_point_names.markers.append(marker_point_name)

        marker_points.points = points_array
        marker_points.colors = points_colors

        # Publish the marker points and names
        self.publisher_workspace_area.publish(marker_points)
        self.publisher_workspace_area_names.publish(marker_point_names)

    


    #def publish_payload_force(self, frames_names : np.ndarray[str]):
        



def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()