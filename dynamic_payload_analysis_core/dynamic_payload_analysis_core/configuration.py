
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




import numpy as np


class Configuration:
    def __init__(self, 
                 joint_positions: np.ndarray = None,
                 end_effector_pose: np.ndarray = None,
                 tau: np.ndarray = None,
                 tree_id: int = None,
                 selected_joint_id: int = None,
                 maximum_payload: float = 0.0 ):
        """
        Initialize a Configuration instance.

        Args:
            joint_positions (np.ndarray): Array of joint positions.
            end_effector_pose (np.ndarray): Pose of the end effector.
            tau (np.ndarray): Array of torque values.
            tree_id (int): Identifier for the tree.
            selected_joint_id (int): The ID of the selected joint.
            maximum_payload (float): The maximum payload value.
        """
        self.joint_positions = joint_positions  # numpy array of joint positions
        self.end_effector_pose = end_effector_pose  # numpy array representing the pose

        self.tau = tau # Placeholder for torque attribute
        self.tree_id = tree_id   # Placeholder for tree ID attribute
        self.selected_joint_id = selected_joint_id  # Placeholder for selected joint ID attribute

        self.maximum_payload = maximum_payload  # Placeholder for maximum payload attribute


    def set_tau(self, tau: np.ndarray):
        """
        Set the torque for this configuration.

        Args:
            tau (np.ndarray): Array of torque values.
        """
        self.tau = tau

    def set_tree_id(self, tree_id: int):
        """
        Set the tree ID for this configuration.

        Args:
            tree_id (int): Identifier for the tree.
        """
        self.tree_id = tree_id

    
    def set_selected_joint_id(self, joint_id: int):
        """
        Set the selected joint ID for this configuration.

        Args:
            joint_id (int): The ID of the selected joint.
        """
        self.selected_joint_id = joint_id

    def set_maximum_payload(self, payload: float):
        """
        Set the maximum payload for this configuration.
        Args:
            payload (float): The maximum payload value.
        """
        self.maximum_payload = payload

    