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
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from dynamic_payload_analysis_core.core import TorqueCalculator
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_payload_analysis_ros.menu_visual import MenuPayload


#TODO : Add payload mass selection in the menu
#TODO : Remove mimim joints in label visualization


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

        # subscription to joint states
        self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.robot = None

        # frame where the external force is applied
        self.frame_id = None

        self.external_force = None

        self.get_logger().info('Robot description subscriber node started')


    def robot_description_callback(self, msg):
        self.get_logger().info('Received robot description')

        self.robot = TorqueCalculator(robot_description = msg.data)
        self.robot.print_active_joint()
        self.robot.print_frames()
        
        # Add the frame to the menu for payload selection
        for frame in self.robot.get_active_frames():
            self.menu.insert(frame)

        # self.robot.print_configuration()


    def joint_states_callback(self, msg):
        if self.robot is not None:
            positions = list(msg.position)
            name_position = list(msg.name)
            v = msg.velocity if msg.velocity else self.robot.get_zero_velocity()
            
            q = self.robot.set_position(positions, name_position)
            a = self.robot.get_zero_acceleration()

                    
            # create the array with only the checked frames (with external force applied)
            self.checked_frames = np.array([check_frame["name"] for check_frame in self.menu.get_item_state() if check_frame['checked']])

            # if there are no checked frames, set the external force to None
            if len(self.checked_frames) != 0:
                self.external_force = self.robot.create_ext_force(mass=4.0, frame_name=self.checked_frames, q=q)
            else:
                self.external_force = None

            # compute the inverse dynamics
            tau = self.robot.compute_inverse_dynamics(q, v, a, extForce=self.external_force)

            # check the effort limits
            status_efforts = self.robot.check_effort_limits(tau)
            # print the torques
            self.robot.print_torques(tau)

            # get the active frames and joints positions
            frames = self.robot.get_active_frames()

            # get the positions of the joints where the torques are applied
            joints_position = self.robot.get_joints_placements(q)

            # Publish the torques in RViz
            self.publish_label_torques(tau, status_torques=status_efforts ,joints_position=joints_position)

            # publish the external force as arrows in RViz
            self.publish_payload_force(self.menu.get_item_state())


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


    
    def publish_payload_force(self, frames_names : np.ndarray[str]):
        """
        Publish the gravity force on the frame with id `id_force`.

        Args:
            frame_names : The frames where the external forces is applied.
        """
        external_force_array = MarkerArray()
        
        for frame in frames_names:

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