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
from dynamic_payload_analysis_ros.menu_visual import TorqueVisualizationType





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
        self.publisher_workspace_area = self.create_publisher(MarkerArray, '/workspace_area', 10)

        # Publisher for point names in the workspace area
        self.publisher_workspace_area_names = self.create_publisher(MarkerArray, '/workspace_area_names', 10)

        # subscription to joint states
        self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # publisher for the joint states
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        
        # variable to store the object of the TorqueCalculator
        self.robot = None

        # variable to store external force applied on the robot
        self.external_force = None

        # variable to store checked frames in the menu where a payload is applied
        self.checked_frames = []

        # variable to store masses applied on the frames
        self.masses = None

        # variable to store the currente selected configuration from the workspace menu
        self.valid_configurations = None

        # variable to store if there is a selected configuration from the workspace menu to visualize
        self.selected_configuration = None

        # timer to compute the valid workspace area 
        self.timer_workspace_calculation = self.create_timer(1, self.workspace_calculation)
        # timer to publish the selected configuration in joint states
        self.timer_publish_configuration = self.create_timer(2, self.publish_selected_configuration)
        # timer to calculate the external forces based on the checked frames in the menu
        self.timer_update_checked_frames = self.create_timer(0.5, self.update_checked_frames)
        # timer to publish the external forces as arrows in RViz
        self.timer_publish_force = self.create_timer(1, self.publish_payload_force)

        self.get_logger().info('Robot description subscriber node started')


    def robot_description_callback(self, msg):
        """
        Callback function for the robot description topic.
        This function initializes the TorqueCalculator with the robot description received from the topic.
        """

        self.get_logger().info('Received robot description')

        self.robot = TorqueCalculator(robot_description = msg.data)
        self.robot.print_active_joint()
        self.robot.print_frames()
        
        # Add the frame to the menu for payload selection
        for frame in self.robot.get_active_frames():
            self.menu.insert_frame(frame)

        # self.robot.print_configuration()


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
        Callback for timer to compute the valid workspace area.
        """
        # if the user choose to compute the workspace area then compute the valid configurations
        if self.menu.get_workspace_state():
            self.valid_configurations = self.robot.get_valid_workspace(2, 0.20, "arm_left_7_joint", "arm_right_7_joint", self.masses, self.checked_frames)

            # compute the maximum payloads for the valid configurations
            self.valid_configurations = self.robot.compute_maximum_payloads(self.valid_configurations)
            
            # insert the valid configurations in the menu
            self.menu.insert_dropdown_configuration(self.valid_configurations)

            # clear all the workspace area markers
            self.clear_workspace_area_markers()

            # set the workspace state to False to stop the computation
            self.menu.set_workspace_state(False)

        # if there are valid configurations, publish the workspace area
        if self.valid_configurations is not None:
            # publish the workspace area
            self.publish_workspace_area()
            
            

    def publish_selected_configuration(self):
        """
        Timer to publish the selected configuration.
        This will publish the joint states of the selected configuration in the menu.
        """
        # get the selected configuration from the menu
        self.selected_configuration = self.menu.get_selected_configuration()
        
        # if there is a selected configuration, publish the joint states based on the valid configurations calculated previously
        if self.selected_configuration is not None:
            configs = self.robot.get_position_for_joint_states(self.valid_configurations[self.selected_configuration]["config"])
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            
            joint_state.name = [joint["joint_name"] for joint in configs]
            joint_state.position = [joint["q"] for joint in configs]
            #joint_state.position =
            self.publisher_joint_states.publish(joint_state)

        else:
            if self.robot is not None:
                # if there is no selected configuration, publish the joint states with zero positions
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()

                joint_state.name = self.robot.get_joints().tolist()
                zero_config = self.robot.get_position_for_joint_states(self.robot.get_zero_configuration())
                joint_state.position = [joint["q"] for joint in zero_config]

                self.publisher_joint_states.publish(joint_state)

    
    def update_checked_frames(self):
        """
        Function to update the external forces based on the checked frames in the menu.    
        """
        # create the array with only the checked frames (with external force applied)
        self.checked_frames = np.array([check_frame["name"] for check_frame in self.menu.get_item_state() if check_frame['checked']])
        
        if len(self.checked_frames) != 0:
            # create the array with the masses of the checked frames
            self.masses = np.array([check_frame["payload"] for check_frame in self.menu.get_item_state() if check_frame['checked']])
        else:
            # if there are no checked frames, set the masses to None
            self.masses = None


    def joint_states_callback(self, msg):
        """
        Callback function for the joint states topic.
        """
        if self.robot is not None:
            # if you are not using the calculated configuration from workspace, you can use the joint states to compute the torques because you don't have already the computed torques
            if self.selected_configuration is None:
                self.positions = list(msg.position)
                self.name_positions = list(msg.name)
                v = msg.velocity if msg.velocity else self.robot.get_zero_velocity()
                
                # set the positions based on the joint states
                q = self.robot.set_position(self.positions, self.name_positions)
                a = self.robot.get_zero_acceleration()

                # if there are no checked frames, set the external force to None
                if len(self.checked_frames) != 0:
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
                joints_position, offset_z = self.robot.get_joints_placements(q)

                # Publish the torques in RViz
                self.publish_label_torques(tau, status_torques=status_efforts ,joints_position=joints_position)

            else:
                # if you are using the calculated configuration from workspace, you can use the valid configurations to visualize the torques labels

                # get the positions of the joints where the torques are applied based on 
                joints_position, offset_z = self.robot.get_joints_placements(self.valid_configurations[self.selected_configuration]["config"])
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
        

    def publish_workspace_area(self):
        """
        Publish the workspace area in RViz using points and labels for the end points.

        """
        # Create a MarkerArray to visualize the number of configuration of a specific point in the workspace
        marker_point_names = MarkerArray()

        # Create a Marker for the workspace area using points
        marker_points = MarkerArray()

        # calculate the maximum torques for each joint in the current valid configurations for each arm only if the user selected the max current torque visualization
        if self.menu.get_torque_color() == TorqueVisualizationType.MAX_CURRENT_TORQUE:
            max_torque_for_joint_left, max_torque_for_joint_right = self.robot.get_maximum_torques(self.valid_configurations)
        

        

        cont = 0
        # Iterate through the valid configurations and create markers
        for i, valid_config in enumerate(self.valid_configurations):
            
            # create the label for the end point (end effector position) of the valid configuration
            marker_point_name = Marker()
            marker_point_name.header.frame_id = "base_link"
            marker_point_name.header.stamp = self.get_clock().now().to_msg()

            marker_point_name.ns = f"workspace_area_{valid_config['arm']}_arm"
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
            
            # get the joint positions for the valid configuration
            joint_positions, offset_z = self.robot.get_joints_placements(valid_config["config"])

            # get the normalized torques for the valid configuration with current target limits for color visualization
            if self.menu.get_torque_color() == TorqueVisualizationType.TORQUE_LIMITS:
                norm_joints_torques = self.robot.get_normalized_torques(valid_config["tau"])
            else:
                # if the user selected the max torque, use the maximum torques for the joint
                if valid_config["arm"] == "left":
                    norm_joints_torques = self.robot.get_normalized_torques(valid_config["tau"],max_torque_for_joint_left)
                else:
                    norm_joints_torques = self.robot.get_normalized_torques(valid_config["tau"],max_torque_for_joint_right)
            
            # insert points related to the end effector position in the workspace area and with color based on the normalized torques for each joint
            for joint_pos,tau in zip(joint_positions,norm_joints_torques):
                # print only the points for the corrisponding arm, in this way we can visualize the workspace area for each arm separately and avoid confusion
                if valid_config["arm"] in joint_pos["name"] : 
                    point = Marker()
                    point.header.frame_id = "base_link"
                    point.header.stamp = self.get_clock().now().to_msg()
                    point.ns = joint_pos["name"]
                    point.id = cont
                    point.type = Marker.SPHERE
                    point.action = Marker.ADD
                    point.scale.x = 0.03  # Size of the sphere
                    point.scale.y = 0.03
                    point.scale.z = 0.03
                    
                    point.pose.position.x = valid_config["end_effector_pos"][0]
                    point.pose.position.y = valid_config["end_effector_pos"][1]
                    point.pose.position.z = valid_config["end_effector_pos"][2] - offset_z
                    point.pose.orientation.w = 1.0
                    point.color.a = 1.0  # Alpha
                    point.color.r = tau  # Red
                    point.color.g = 1 - tau  # Green
                    point.color.b = 0.0  # Blue

                    cont += 1

                    # Add the point to the points array
                    marker_points.markers.append(point)
            
            # Add the marker point name to the marker point names array
            marker_point_names.markers.append(marker_point_name)


        # get the unified torque for the valid configurations
        unified_configurations_torque = self.robot.get_unified_configurations_torque(self.valid_configurations)

        # insert points related to the end effector position in the workspace area and with color based on the normalized torque for each configuration
        # this is used to visualize the workspace area with the unified torques for each configuration
        for i, unified_config in enumerate(unified_configurations_torque):
            marker_point = Marker()
            marker_point.header.frame_id = "base_link"
            marker_point.header.stamp = self.get_clock().now().to_msg()
            marker_point.ns = f"unified_torque_workspace_{unified_config['arm']}_arm"
            marker_point.id = i
            marker_point.type = Marker.SPHERE
            marker_point.action = Marker.ADD
            marker_point.scale.x = 0.03  # Size of the sphere
            marker_point.scale.y = 0.03
            marker_point.scale.z = 0.03
            marker_point.pose.position.x = unified_config["end_effector_pos"][0]
            marker_point.pose.position.y = unified_config["end_effector_pos"][1]
            marker_point.pose.position.z = unified_config["end_effector_pos"][2] - offset_z # Offset to avoid overlap with the sphere
            marker_point.pose.orientation.w = 1.0
            marker_point.color.a = 1.0  # Alpha

            # Color based on the normalized torques
            marker_point.color.r = unified_config["norm_tau"]
            marker_point.color.g = 1 - unified_config["norm_tau"]
            marker_point.color.b = 0.0  # Blue

            # Add the marker point to the points array
            marker_points.markers.append(marker_point)


        # Publish the marker points and names
        self.publisher_workspace_area.publish(marker_points)
        self.publisher_workspace_area_names.publish(marker_point_names)
        

    def clear_workspace_area_markers(self):
        """
        Clear the workspace area markers in RViz.
        This will publish an empty MarkerArray to remove the previous markers.
        """
        # create an empty MarkerArray to clear the markers with a DELETEALL action
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)

        # Publish the empty MarkerArray to clear the markers
        self.publisher_workspace_area.publish(marker_array_msg)
        self.publisher_workspace_area_names.publish(marker_array_msg)




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