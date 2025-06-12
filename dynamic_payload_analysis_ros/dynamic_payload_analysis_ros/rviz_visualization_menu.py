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
        self.publisher_force = self.create_publisher(WrenchStamped, '/external_force', 10)

        # Publisher for RViz visualization of torques
        self.publisher_rviz_torque = self.create_publisher(MarkerArray, '/torque_visualization', 10)

        # subscription to joint states
        self.joint_states_subscription = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.robot = None

        # frame where the external force is applied
        self.frame_id = "arm_left_7_link"

        self.external_force = None

        self.get_logger().info('Robot description subscriber node started')


    def robot_description_callback(self, msg):
        self.get_logger().info('Received robot description')

        self.robot = TorqueCalculator(robot_description = msg.data)
        self.robot.print_active_joint()
        self.robot.print_frames()
        
        # Add the frame to the menu for selecting where to apply the payload
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


            # Compute inverse dynamics with external force
            for frame in self.menu.get_item_state():
                if frame['checked']:
                    self.frame_id = frame['name']
                    self.external_force = self.robot.create_ext_force(mass=4.0, frame_name=self.frame_id, q=q)
                

            tau = self.robot.compute_inverse_dynamics(q, v, a, extForce=self.external_force)

            status_efforts = self.robot.check_effort_limits(tau)
            self.robot.print_torques(tau)
            frames = self.robot.get_active_frames()
            joints_position = self.robot.get_joints_placements(q)

            # Publish the torques in RViz
            self.publish_rviz_torque(tau, status_torques=status_efforts ,joints_position=joints_position)

            # publish the external force
            if self.external_force is not None:
                self.publish_payload_force(self.frame_id, self.external_force)


    def publish_rviz_torque(self, torque: np.ndarray, status_torques : np.ndarray, joints_position: np.ndarray):
        """
        Publish the torque on the robot in RViz.

        Args:
            torque (np.ndarray): The torque to be published
            status_torques (np.ndarray): The status of the torques, True if the torque is within the limits, False otherwise
            joints_position (np.ndarray): The positions of the joints where the torques are applied
        """
        marker_array = MarkerArray()
        for i, (t, joint) in enumerate(zip(torque, joints_position)):
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


    
    def publish_payload_force(self, frame_id : str, external_force: np.ndarray):
        """
        Publish the gravity force on the frame with id `id_force`.

        Args:
            frame_id (str): The frame where the external force is applied.
            external_force (np.ndarray): The external force to be published
        """
        id_force = self.robot.get_parent_joint_id(frame_id)
        position = self.robot.get_joint_placement(id_force)

        # force = Marker()
        # force.header.frame_id = frame_id
        # force.header.stamp = self.get_clock().now().to_msg()
        # force.ns = "external_force"
        # force.id = id_force
        # force.type = Marker.ARROW
        # force.action = Marker.ADD
        # force.scale.x = 0.20   
        # force.scale.y = 0.05  
        # force.scale.z = 0.05 
        # force.color.a = 1.0  # Alpha
        # force.color.r = 0.0  # Red
        # force.color.g = 0.0
        # force.color.b = 1.0  # Blue
        # # Set the direction of the arrow based on the external force
        # force.pose.orientation.x = external_force[id_force].linear[0]
        # force.pose.orientation.y = external_force[id_force].linear[1]
        # force.pose.orientation.z = external_force[id_force].linear[2]
        # force.pose.orientation.w = 0.0  # Set to 1.0 for a straight arrow
        # self.publisher_force.publish(force)

        force_msg = WrenchStamped()

        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.header.frame_id = frame_id

        force_msg.wrench.force.x = external_force[id_force].linear[0] * 0.01
        force_msg.wrench.force.y = external_force[id_force].linear[1] * 0.01
        force_msg.wrench.force.z = external_force[id_force].linear[2] * 0.01
        force_msg.wrench.torque.x = external_force[id_force].angular[0]
        force_msg.wrench.torque.y = external_force[id_force].angular[1]
        force_msg.wrench.torque.z = external_force[id_force].angular[2]
        


        self.publisher_force.publish(force_msg)

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