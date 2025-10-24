
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
from dynamic_payload_analysis_core.configuration import Configuration

class Subtree:
    def __init__(self, id : int,
                 link_names : list[str] = None,
                 joint_names : list[str] = None,
                 joint_ids : list[int] = None,
                 selected_joint_id : int = None):
        """
        Initialize a Subtree instance.
        
        Args:
            name (str): Name of the subtree.
            tree_id (int): Identifier for the tree this subtree belongs to.
            link_names (list[str], optional): List of link names in the subtree.
            joint_names (list[str], optional): List of joint names in the subtree.
            joint_ids (list[int], optional): List of joint IDs in the subtree.
            selected_joint_id (int, optional): The ID of the selected joint in the subtree.
        """
        self.id = id

        self.selected_joint_id = selected_joint_id
        self.configurations = np.empty((0, 0))
        self.link_names = link_names if link_names is not None else []
        self.joint_names = joint_names if joint_names is not None else []
        self.joint_ids = joint_ids if joint_ids is not None else []
        
        self.tip_link_name = link_names[-1] if link_names is not None else None
        self.tip_joint_name = joint_names[-1] if joint_names is not None else None
        self.tip_joint_id = joint_ids[-1] if joint_ids is not None else None

        self.calculated_range = None  # Placeholder for calculated range attribute
        self.resolution = None  # Placeholder for resolution attribute
        self.iterations = None  # Placeholder for iterations attribute


    def set_selected_joint_id(self, joint_id : int):
        """
        Set the selected joint ID for this subtree.
        """
        self.selected_joint_id = joint_id
    
    def add_configuration(self, configuration : np.ndarray):
        """
        Add a configuration to this subtree.
        
        Args:
            configuration (np.array[Configuration]): The configuration to add.
        """
        if self.configurations.size == 0:
            self.configurations = np.array([configuration])
        else:
            self.configurations = np.append(self.configurations, [configuration], axis=0)


    def check_joint_in_subtree(self, joint_id : int) -> bool:
        """
        Check if a given joint ID is part of this subtree.
        
        Args:
            joint_id (int): The joint ID to check.
        """

        return joint_id in self.joint_ids
    

    def set_calculation_settings(self, calculated_range : float,
                                  resolution : float,
                                  iterations : int):
        """
        Set the current calculation settings for this subtree.
        
        Args:
            calculated_range (float): The calculated range.
            resolution (float): The resolution.
            iterations (int): The number of iterations.
        """
        self.calculated_range = calculated_range
        self.resolution = resolution
        self.iterations = iterations
        
    

    