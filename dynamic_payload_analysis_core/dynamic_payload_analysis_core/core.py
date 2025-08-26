
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


import pinocchio as pin
import numpy as np
import math
from typing import Union
from pathlib import Path
from numpy.linalg import norm, solve
import tempfile
import os
from urdf_parser_py.urdf import URDF


class TorqueCalculator:
    def __init__(self, robot_description : str):
        """
        Initialize the Torques_calculator with the URDF model or XML format provided by robot_description topic.
        
        :param robot_description: Robot description in XML format provided by /robot_description topic.
        """

        # Load the robot model from path or XML string
        if isinstance(robot_description, str):
            self.model = pin.buildModelFromXML(robot_description)
            
            # compute mimic joints 
            self.compute_mimic_joints(robot_description)

            # get the root joint name
            self.root_name = self.get_root_joint_name(robot_description)

            # create temporary URDF file from the robot description string
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as temp_file:
                    temp_file.write(robot_description)
                    temp_urdf_path = temp_file.name

            self.geom_model = pin.buildGeomFromUrdf(self.model,temp_urdf_path,pin.GeometryType.COLLISION)
            
            # Add collisition pairs
            self.geom_model.addAllCollisionPairs()

            os.unlink(temp_urdf_path)

        else:
            raise ValueError("robot_description must be a string containing the URDF XML or the path to the URDF file")
        
        # create data for the robot model
        self.data = self.model.createData()
        self.geom_data = pin.GeometryData(self.geom_model)

        # get the default collisions in the robot model to avoid take them into account in the computations
        self.default_collisions = self.compute_static_collisions()

        # compute main trees of the robot model
        self.compute_subtrees()

        # array to store all analyzed points
        self.analyzed_points = np.array([], dtype=object)

        # array to store all configurations for the robot model
        self.configurations = np.array([], dtype=object)

        # create items for each tree in the robot model
        for tree in self.subtrees:
            self.configurations = np.append(self.configurations, {"tree_id": tree["tree_id"], "configurations": None, "selected_joint_id" : None})


    def get_root_joint_name(self, robot_description: str) -> str:
        """
        Get the root joint name from the robot description XML string.
        
        :param robot_description: Robot description in XML format provided by /robot_description topic.
        """
        try:
            robot = URDF.from_xml_string(robot_description)
            root_name = robot.get_root()
        except Exception as e:
            print(f"Error parsing URDF xml: {e}")
            root_name = None

        return root_name

    
    def compute_mimic_joints(self, urdf_xml):
        """
        Function to find all mimic joints with mimicked joints and ids.

        Args:
            urdf_xml (str): The string from robot_description topic.
        """
        try:
            robot = URDF.from_xml_string(urdf_xml)
        except:
            print(f"Error parsing URDF xml")

        self.mimic_joint_names = []
        self.mimicked_joint_names = []

        # Iterate through all joints in the robot model to find mimic joints
        for joint in robot.joints:
            if joint.mimic:
                # Add the mimic joint name to the list
                self.mimic_joint_names.append(joint.name)
                # Add the mimicked joint name to the list
                self.mimicked_joint_names.append(joint.mimic.joint)

        # create lists of joint ids for mimic and mimicked joints
        self.mimic_joint_ids = [self.model.getJointId(name) for name in self.mimic_joint_names]
        self.mimicked_joint_ids = [self.model.getJointId(name) for name in self.mimicked_joint_names]


    def compute_static_collisions(self):
        """
        Compute the static collisions for the robot model.
        This method is used to compute the collisions in the robot model in the zero configuration.
        """
        
        # array to store the collision pairs
        collision_pairs = []

        # Compute all the collisions
        pin.computeCollisions(self.model, self.data, self.geom_model, self.geom_data, self.get_zero_configuration(), False)
                              
        # Print the status of collision for all collision pairs
        for k in range(len(self.geom_model.collisionPairs)):
            cr = self.geom_data.collisionResults[k]
            cp = self.geom_model.collisionPairs[k]
            if cr.isCollision():
                print(f"Collision between {cp.first} and {cp.second} detected.")
                collision_pairs.append((cp.first, cp.second, k))

        return collision_pairs
        
    
    def compute_subtrees(self):
        """
        Compute the sub-trees of the robot model.
        This method is used to compute the sub-trees of the robot model
        """
        
        tip_joints = []
        for id in range(0, self.model.njoints):
          if len(self.model.subtrees[id]) == 1:
            tip_joints += [id]
        
        self.subtrees = np.array([], dtype=object)
        cont = 0
        for i, jointID in enumerate(tip_joints):
            joint_tree_ids = self.get_filtered_subtree(jointID)
            
            # insert the sub-tree only if the tip joint is not already in the sub-trees
            tip_joint_already_exists = False
            for existing_tree in self.subtrees:
                if existing_tree["tip_joint_id"] == joint_tree_ids[-1]:
                    tip_joint_already_exists = True
                    break

            if not tip_joint_already_exists:
                # get the link names in the sub-tree
                link_names = self.get_links_from_tree(joint_tree_ids)
                # get the joint names in the sub-tree
                joint_names = [self.model.names[joint_id] for joint_id in joint_tree_ids]

                self.subtrees = np.append(self.subtrees, {"tree_id": cont, "link_names": link_names ,"joint_names": joint_names, "joint_ids": joint_tree_ids,"tip_link_name": link_names[-1], "tip_joint_id": joint_tree_ids[-1], "selected_joint_id": None})
                cont += 1
    

    def get_filtered_subtree(self, current_tip_id : int) -> np.ndarray:
        """
        Filter the sub-trees of the robot based on the mimic joints and mimicked joints.
        If the current tip joint is not a mimic joint, the subtree is returned as is.
        
        :param current_tip_id: Id of the current tip joint to filter the subtree.
        :return: Filtered tree with tip joint based on the mimicked joint of the mimic joint.
        """
        # find mimicked joint of the current tip joint
        if current_tip_id in self.mimic_joint_ids:
            # get the index of the mimic joint in the mimic_joint_ids
            mimic_joint_index = self.mimic_joint_ids.index(current_tip_id)
            # get the mimicked joint id
            mimicked_joint_id = self.mimicked_joint_ids[mimic_joint_index]
            
            # filter the subtree to include only the mimicked joint and its children
            filtered_subtree = self.model.supports[mimicked_joint_id].tolist()
        else:
            # if the current tip joint is not a mimic joint, return the subtree as is
            filtered_subtree = self.model.supports[current_tip_id].tolist()

        # remove universe joint
        filtered_subtree = filtered_subtree[1:]
        
        return filtered_subtree


    def compute_inverse_dynamics(self, q : np.ndarray , qdot : np.ndarray, qddot : np.ndarray, extForce : np.ndarray = None) -> np.ndarray:
        """
        Compute the inverse dynamics torque vector.
        
        :param q: Joint configuration vector.
        :param qdot: Joint velocity vector.
        :param qddot: Joint acceleration vector.
        :param extForce: External forces vector applied to the robot model.
        :return: Torques vector
        """

        # basic equation for inverse dynamics : M(q) *a + b = tau + J(q)_t * extForce --> tau = M(q) * a + b - J(q)_t * extForce

        if extForce:
            tau = pin.rnea(self.model, self.data, q, qdot, qddot, extForce)
        else:
            tau = pin.rnea(self.model, self.data, q, qdot, qddot)
        
        if tau is None:
            raise ValueError("Failed to compute torques")
    
        return tau
    

    def create_ext_force(self, masses : Union[float, np.ndarray] , frame_name : Union[str | np.ndarray], q : np.ndarray) -> list:
        """
        Create external forces vector based on the masses and frame ID.
        The resulting vector will contain the force applied to the specified frame and with the local orientation of the parent joint.
        
        :param masses (float, np.ndarray) : Mass of the object to apply the force to or vector with masses related to frames names.
        :param frame_name(str , np.ndarray) : Frame name where the force is applied or vector of frame names where the forces is applied.
        :param q: Joint configuration vector.
        :return: External force vector.
        """
        if isinstance(masses, float):
            if masses < 0:
                raise ValueError("Mass must be a positive value")
        
        if frame_name is None:
            raise ValueError("Frame name must be provided")
        
        #if frame_name not in self.model.frames:
        #    raise ValueError(f"Frame name '{frame_name}' not found in the robot model")

        # assumption made : the force is applied to the joint
        
        # Initialize external forces array
        fext = [pin.Force(np.zeros(6)) for _ in range(self.model.njoints)]
        
        self.update_configuration(q)

        # Check if frame_name is a single string or an array of strings
        if isinstance(frame_name, str):
             # Get the frame ID and joint ID from the frame name
            frame_id = self.model.getFrameId(frame_name)
            joint_id = self.model.frames[frame_id].parentJoint

            # force expressed in the world frame (gravity force in z axis)
            ext_force_world = pin.Force(np.array([0.0, 0.0, masses * -9.81]), np.array([0.0, 0.0, 0.0])) 

            # placement of the frame in the world frame
            #frame_placement = self.data.oMf[frame_id]
            #print(f"Frame placement: {frame_placement}")
            
            # Convert the external force expressed in the world frame to the orientation of the joint frame where the force is applied
            fext[joint_id] = self.data.oMi[joint_id].actInv(ext_force_world)
            # Zero out the last 3 components (torques) of the force to ensure only the force in z axis (gravity force) is applied
            fext[joint_id].angular = np.zeros(3) # TODO : make it more efficient 

        else:
            for mass, frame in zip(masses,frame_name):
                frame_id = self.model.getFrameId(frame)
                joint_id = self.model.frames[frame_id].parentJoint

                # force expressed in the world frame (gravity force in z axis)
                ext_force_world = pin.Force(np.array([0.0, 0.0, mass * -9.81]), np.array([0.0, 0.0, 0.0]))
                # Convert the external force expressed in the world frame to the orientation of the joint frame where the force is applied
                fext[joint_id] = self.data.oMi[joint_id].actInv(ext_force_world)
                # Zero out the last 3 components (torques) of the force to ensure only the force in z axis (gravity force) is applied
                fext[joint_id].angular = np.zeros(3)

        return fext


    def update_configuration(self, q : np.ndarray):
        """
        Update the robot model configuration with the given joint configuration vector.
        
        :param q: Joint configuration vector.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
    


    def compute_inverse_kinematics_optik(self, q : np.ndarray, end_effector_position: np.ndarray) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot model using the Optik library.
        
        :param q: current joint configuration vector.
        :param end_effector_position: Position of the end effector in the world frame [rotation matrix , translation vector].
        :return: Joint configuration vector that achieves the desired end effector position.
        """
        # TODO : It doees not work with the current version of the library
        
        # Compute the inverse kinematics
        sol = self.ik_model.ik(self.ik_config, end_effector_position, q)
        
        return sol
    


    def compute_inverse_kinematics_ikpy(self, q : np.ndarray, end_effector_position: np.ndarray) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot model using the ikpy library.
        
        :param q: current joint configuration vector.
        :param end_effector_position: Position of the end effector in the world frame [rotation matrix , translation vector].
        :return: Joint configuration vector that achieves the desired end effector position.
        """
        # TODO : It doees not work with the current version of the library
        
        # Compute the inverse kinematics
        sol = self.ik_model.inverse_kinematics(end_effector_position)
        
        return sol



    def compute_inverse_kinematics(self, q : np.ndarray, end_effector_position: np.ndarray, joint_id : str) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot model with joint limits consideration.
        
        :param q: Current joint configuration vector.
        :param end_effector_position: Position of the end effector in the world frame [rotation matrix , translation vector].
        :param joint_id: Id of the end effector joint.
        :return: Joint configuration vector that achieves the desired end effector position.
        """

        # Set parameters for the inverse kinematics solver
        eps = 1e-2 # reduce for more precision
        IT_MAX = 500 # Maximum number of iterations
        DT = 1e-1 
        damp = 1e-12

        i = 0
        while True:
            self.update_configuration(q)
            iMd = self.data.oMi[joint_id].actInv(end_effector_position) # Get the transformation from the current end effector pose to the desired pose
            
            err = pin.log(iMd).vector  # compute the error in the end effector position
            if norm(err[:3]) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break

            J = pin.computeJointJacobian(self.model, self.data, q, joint_id)  # compute the Jacobian of current pose of end effector
            #J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            J = -J[:3, :] 

            # compute the inverse kinematics v = -J^T * (J * J^T + damp * I)^-1 * err
            v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err[:3]))
            
            # Apply joint limits by clamping the resulting configuration
            q_new = pin.integrate(self.model, q, v * DT)
            # Ensure the new configuration is within joint limits
            q_new = np.clip(q_new, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            
            # Check if we're hitting joint limits and reduce step size if needed
            if np.allclose(q_new, q, atol=1e-6):
                DT *= 0.5  # Reduce step size if no progress due to joint limits
                if DT < 1e-6:  # Minimum step size threshold
                    success = False
                    break
            
            q = q_new

            # if not i % 10:
            #     print(f"{i}: error = {err.T}")
            i += 1
        
        if success:
            print(f"Convergence achieved! in {i} iterations")
            return q
        else:
            print(
                "\n"
                "Warning: the iterative algorithm has not reached convergence to the desired precision"
            )
            return None  # Return None if convergence is not achieved
        
            

    def compute_all_configurations(self, range : int, resolution : int, end_joint_id) -> np.ndarray:
        """
        Compute all configurations for the robot model within a specified range.
        
        :param range (int): Range as side of a square where in the center there is the actual position of end effector.
        :param resolution (int): Resolution of the grid to compute configurations.
        :param end_joint_id (str): Id of the end effector joint selected in the tree.
        :return : Array of joint configurations that achieve the desired end effector position.
        """
        
        if range <= 0:
            raise ValueError("Range must be a positive value")
        
        # Get the current joint configuration
        q = self.get_zero_configuration()

        #id_end_effector = self.model.getJointId(end_effector_name)
        # Get the current position of the end effector
        #end_effector_pos = self.data.oMi[id_end_effector]
        
        # Create an array to store all configurations
        configurations = []
        
        # Iterate over the range to compute all configurations
        for x in np.arange(-range, range , resolution):
            for y in np.arange(-range, range , resolution):
                for z in np.arange(-range/2, range , resolution):
                    target_position = pin.SE3(np.eye(3), np.array([x, y, z]))
                    new_q = self.compute_inverse_kinematics(q, target_position, end_joint_id)
                     
                    if new_q is not None:
                        q = new_q
                        # store the valid configuration and the position of the end effector relative to that configuration
                        configurations.append({"config" : new_q, "end_effector_pos": target_position.translation}) 
        
        return np.array(configurations, dtype=object)
    


    def verify_configurations(self, configurations: np.ndarray, masses : np.ndarray, checked_frames : np.ndarray, tree_id: int, selected_joint_id: int) -> np.ndarray:
        """
        Verify the configurations to check if they are valid.
        
        :param configurations: Array of joint configurations to verify for the left arm.
        :param masses (np.ndarray): Array of masses to apply to the robot model.
        :param checked_frames (np.ndarray): Array of frame names where the external forces are applied.
        :param tree_id (int): Identifier of the tree to verify the configurations for.
        :param selected_joint_id (int): Identifier of the selected joint in the tree to verify the configurations for.
        :return: Array of valid configurations with related torques in format: [{"config", "end_effector_pos, "tau", "tree_id","selected_joint_id" }].
        """
        # TODO : get the tree_id from the sub tree array instead of passing it as parameter
        valid_configurations = []
        
        # check valid configurations for left arm
        for q in configurations:
            # Update the configuration of the robot model
            self.update_configuration(q["config"])
            
            if masses is not None and checked_frames is not None:
                # Create external forces based on the masses and checked frames
                ext_forces = self.create_ext_force(masses, checked_frames, q["config"])
                # Compute the inverse dynamics for the current configuration
                tau = self.compute_inverse_dynamics(q["config"], self.get_zero_velocity(), self.get_zero_acceleration(),extForce=ext_forces)
            else:
                # Compute the inverse dynamics for the current configuration without external forces
                tau = self.compute_inverse_dynamics(q["config"], self.get_zero_velocity(), self.get_zero_acceleration())

            # Check if the torques are within the effort limits 
            if self.check_effort_limits(tau= tau, tree_id= tree_id).all():
                valid = True
                # Compute all the collisions
                pin.computeCollisions(self.model, self.data, self.geom_model, self.geom_data, q["config"], False)

                # Print the status of collision for all collision pairs
                for k in range(len(self.geom_model.collisionPairs)):
                    cr = self.geom_data.collisionResults[k]
                    cp = self.geom_model.collisionPairs[k]

                    if cr.isCollision() and (cp.first, cp.second, k) not in self.default_collisions:
                        print(f"Collision detected between {cp.first} and {cp.second} in the left arm configuration.")
                        valid = False
                        break
                
                if valid:
                    valid_configurations.append({"config" : q["config"], "end_effector_pos" : q["end_effector_pos"], "tau" : tau, "tree_id" : tree_id,"selected_joint_id": selected_joint_id})


        return np.array(valid_configurations, dtype=object)
        

    def get_valid_workspace(self, range : int, resolution : float, masses : np.ndarray, checked_frames: np.ndarray) -> np.ndarray:
        """
        Get the valid workspace of the robot model by computing all configurations within a specified range.
        
        :param range (int): Range as side of a square where in the center there is the actual position of end effector.
        :param resolution (int): Resolution of the grid to compute configurations.
        :param masses (np.ndarray): Array of masses to apply to the robot model.
        :param checked_frames (np.ndarray): Array of frame names where the external forces are applied.
        :return: Array of valid configurations that achieve the desired end effector position in format: [{"config", "end_effector_pos, "tau", "tree_id"}].
        """
        # create the array to store all current valid configurations
        valid_current_configurations = np.array([], dtype=object)

        # compute all configurations for the selected joints of the trees
        for tree,configuration in zip(self.subtrees,self.configurations):
            # if the configurations are not computed or the selected joint ID is not the same as in the tree, compute the configurations
            if configuration["configurations"] is None or configuration["selected_joint_id"] != tree["selected_joint_id"]:
                if tree["selected_joint_id"] is not None:
                    # Compute all configurations for the current tree
                    configuration["configurations"] = self.compute_all_configurations(range, resolution,tree["selected_joint_id"])
                    # Set the selected joint ID to the current tree's selected joint ID
                    configuration["selected_joint_id"] = tree["selected_joint_id"]
                else:
                    pass
                    # if the selected joint ID is None in the tree, I could remove the computed configurations,even though it is not necessary and it could be useful if the 
                    # user selects the joint later 
                
                    
            if configuration["configurations"] is not None and tree["selected_joint_id"] is not None:
                # Verify the configurations to check if they are valid
                valid_configurations = self.verify_configurations(configuration["configurations"], masses, checked_frames, tree["tree_id"], tree["selected_joint_id"])
                
                # Append the valid configurations to the current valid configurations array
                valid_current_configurations = np.append(valid_current_configurations, valid_configurations)
            
            
            

        return valid_current_configurations
    

    def compute_maximum_payloads(self, configs : np.ndarray):
        """
        Compute the maximum payload for each provided configuration and return the results with the configs updated with the maximum payload as a new value.
        
        :param configs: Array of configurations , format {"config", "end_effector_pos", "tau", "tree_id", "max_payload" }     
        """
        for config in configs:
            config["max_payload"] = self.find_max_payload_binary_search(config, payload_min=0.0, payload_max=15, resolution=0.01)
        
        return configs


    def find_max_payload_binary_search(self, config : np.ndarray, payload_min : float = 0.0, payload_max : float = 10.0, resolution : float = 0.01):
        """
        Find the maximum payload for a given configuration using binary search.
        
        :param config: Configuration dictionary (must contain 'config' key).
        :param payload_min: Minimum payload to test.
        :param payload_max: Maximum payload to test.
        :param resolution: Desired precision.
        :return: Maximum allowable payload.
        """
        low = payload_min
        high = payload_max
        max_valid = payload_min

        while high - low > resolution:
            mid_payload = (low + high) / 2
            ext_forces = self.create_ext_force(mid_payload, self.get_joint_name(config["selected_joint_id"]), config["config"])
            tau = self.compute_inverse_dynamics(config["config"], self.get_zero_velocity(), self.get_zero_acceleration(), extForce=ext_forces)
            if self.check_effort_limits(tau, config['tree_id']).all():
                max_valid = mid_payload
                low = mid_payload
            else:
                high = mid_payload

        return max_valid

    def compute_forward_dynamics_aba_method(self, q : np.ndarray, qdot : np.ndarray, tau : np.ndarray, extForce : np.ndarray = None) -> np.ndarray:
        """
        Compute the forward dynamics acceleration vector with Articulated-Body algorithm(ABA).
        
        :param q: Joint configuration vector.
        :param qdot: Joint velocity vector.
        :param tau: Joint torque vector.
        :param extForce: External forces vector applied to the robot model.
        :return: Acceleration vector.
        """
        
        # calculate dynamics drift
        if extForce is not None:
            ddq = pin.aba(self.model, self.data, q, qdot, tau, extForce)
        else:
            ddq = pin.aba(self.model, self.data, q, qdot, tau)
        
        return ddq
    

    def compute_jacobian(self, q : np.ndarray, frame_name : str) -> np.ndarray:
        """
        Get the Jacobian matrix for a specific frame in the robot model.
        
        :param q: Joint configuration vector.
        :param frame_name: Name of the frame to compute the Jacobian for.
        :return: Jacobian matrix.
        """
        
        # Get the frame ID
        frame_id = self.model.getFrameId(frame_name)
        joint_id = self.model.frames[frame_id].parentJoint
        
        # Compute the Jacobian
        J = pin.computeJointJacobians(self.model, self.data, q) 
        
        J_frame = pin.getJointJacobian(self.model, self.data, joint_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        if J_frame is None:
            raise ValueError("Failed to compute Jacobian")
        
        return J_frame
    

    def verify_member_tree(self, tree_id: int, joint_id: int) -> bool:
        """
        Verify if the given joint ID is a member of the specified tree.
        
        :param tree_id: Identifier of the tree to verify.
        :param joint_id: Identifier of the joint to verify.
        :return: True if the joint is a member of the tree, False otherwise.
        """
        
        # Check if the joint ID is in the list of joint IDs for the specified tree
        return joint_id in self.subtrees[tree_id]["joint_ids"]


    def get_subtrees(self) -> np.ndarray:
        """
        Get the sub-trees of the robot model.
        
        :return: Array of sub-trees of the robot model.
        """
        return self.subtrees


    def get_links_from_tree(self, joint_ids: np.ndarray | int) -> np.ndarray:
        """
        Get the links from the robot model based on the joint IDs.
        
        :param joint_ids: Array of joint IDs to get the frames for.
        :return: Array of frames corresponding to the joint IDs.
        """
        frames = []
        # If joint_ids is a single integer, convert it to a list
        if isinstance(joint_ids, int):
            joint_ids = [joint_ids]

        for joint_id in joint_ids:
            for link in self.model.frames:
                if link.parentJoint == joint_id and link.type == pin.FrameType.BODY:
                    frames.append(link.name)
                    break
        
        return np.array(frames, dtype=object)
    

    def get_end_effector_position_array(self, x: float, y: float, z: float) -> np.ndarray:
        """
        Get the end effector position as a array.
        
        :param x: X position of the end effector.
        :param y: Y position of the end effector.
        :param z: Z position of the end effector.
        :return: Numpy array representing the end effector position.
        """
        return pin.SE3(np.eye(3), np.array([x, y, z]))

    def get_maximum_torques(self, valid_configs : np.ndarray) -> np.ndarray:
        """
        Get the maximum torques for each joint in all valid configurations.
        
        :param valid_configs: Array of valid configurations with related torques in format: [{"config", "end_effector_pos, "tau"}].
        :return: Arrays of maximum torques for each joint in the current valid configurations for selected trees.
        """
        
        # Get the number of joints
        num_joints = len(valid_configs[0]["tau"])
        
        # array to store the absolute torques for each joint in the current valid configurations for each selected tree
        abs_joint_torques = np.array([], dtype=object)
        
        # get the selected trees from the sub_trees
        selected_trees = [tree for tree in self.subtrees if tree["selected_joint_id"] is not None]
        
        # create an array to store the absolute torques for each joint in the current valid configurations for each selected tree
        for tree in selected_trees:
            # array to store the absolute torques for each joint in the current tree
            abs_torques = np.array([], dtype=object)
            
            for i in range(num_joints):
                # Get the joint torques for the current tree
                abs_torques = np.append(abs_torques ,{"joint" : i ,"abs": [abs(config["tau"][i]) for config in valid_configs if config["tree_id"] == tree["tree_id"]]})

            abs_joint_torques = np.append(abs_joint_torques, {"tree_id": tree["tree_id"], "abs_torques": abs_torques})
            
        # array to store the maximum absolute torques for each joint in the current valid configurations
        max_torques = np.array([], dtype=float)

        # get the maximum absolute torques for each joint in the current valid configurations for each selected tree
        for torques in abs_joint_torques:
            max_tau = np.array([], dtype=object)
            # Get the maximum absolute torque for the current joint
            for tau in torques["abs_torques"]:
                if len(tau["abs"]) > 0:
                    max_tau = np.append(max_tau, max(tau["abs"]))
                else:
                    max_tau = np.append(max_tau, 0.0)
            
            max_torques = np.append(max_torques, {"tree_id": torques["tree_id"], "max_values": max_tau})
        

        return max_torques


    def get_maximum_payloads(self, valid_configs : np.ndarray) -> np.ndarray:
        """
        Get the maximum payloads for all configuration in the corrisponding tree.
        
        :param valid_configs: Array of valid configurations with related torques in format: [{"config", "end_effector_pos, "tau", "tree_id", "max_payload"}].
        :return: Tuple of arrays of maximum payloads for left and right arms.
        """
        max_payloads = np.array([], dtype=float)
        for tree in self.subtrees:
            payloads = [config["max_payload"] for config in valid_configs if config["tree_id"] == tree["tree_id"]]
            if payloads:
                max_payload = max(payloads)
                max_payloads = np.append(max_payloads, {"tree_id": tree["tree_id"], "max_payload": max_payload})

        return max_payloads
            
    
    

    def get_normalized_torques(self, tau : np.ndarray, target_torque : np.ndarray = None, tree_id: int = None) -> np.ndarray:
        """
        Normalize the torques vector to a unified scale.
        
        :param tau: Torques vector to normalize.
        :return: Normalized torques vector.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        norm_tau = []

        # if target_torque is not specified, normalize the torques vector to the effort limits of the robot model
        if target_torque is None:
            # Normalize the torques vector
            for i, torque in enumerate(tau):
                norm_tau.append(abs(torque) / self.model.effortLimit[i])
        else:
            # get the maximum values for the current tree
            max_torque = next(item for item in target_torque if item["tree_id"] == tree_id)
            # Normalize the torques vector to the target torque
            for i, torque in enumerate(tau):
                if max_torque["max_values"][i] != 0:
                    norm_tau.append(abs(torque) / max_torque["max_values"][i])
                else:
                    norm_tau.append(0.0)

        return norm_tau


    def get_normalized_payload(self, payload : np.ndarray, target_payload : float) -> np.ndarray:
        """
        Normalize the torques vector to a unified scale.
        
        :param payload: Maximum payload for a configuration.
        :param target_payload: Target payload to normalize the payload to.
        :return: Normalized payload.
        """
        norm_payload = abs(payload) / target_payload

        return norm_payload    


    def get_unified_configurations_torque(self, valid_configs : np.ndarray) -> np.ndarray | np.ndarray:
        """
        Get a unified sum of torques for all possible configurations of the robot model.
        
        :param q: Joint configuration vector. 
        :param valid_configs: Array of 
        """

        torques_sum = np.array([], dtype=float)
        norm_torques = np.array([], dtype=float)

        # array to store max and min sum of torques for each tree
        max_min_value_torques = np.array([], dtype=float)
        sum = 0.0

        for valid_config in valid_configs:
            # get the joint configuration and torques vector from the valid configuration
            q = valid_config["config"]
            tau = valid_config["tau"]
            
            # calculate the sum of torques for each joint configuration
            for torque in tau:
                #if abs(torque) < 50:
                sum += abs(torque)
                
            torques_sum = np.append(torques_sum, {"sum" : sum, "end_effector_pos" : valid_config["end_effector_pos"], "tree_id" : valid_config["tree_id"]})
            sum = 0.0  # reset the sum for the next configuration


        # get the maximum torque from the sum of torques for all selected trees
        for tree in self.subtrees:
            # Get all sum values for the current tree
            tree_sums = [item["sum"] for item in torques_sum if item["tree_id"] == tree["tree_id"]]
            
            if tree_sums:  # Check if there are any sums for this tree
                max_value = max(tree_sums)
                min_value = min(tree_sums)
            else:
                max_value = 1.0  # Default value if no sums found
                min_value = 0.0  # Default value if no sums found
            
            max_min_value_torques = np.append(max_min_value_torques, {"tree_id": tree["tree_id"], "max_value": max_value, "min_value": min_value})


        # Normalize the torques vector to a unified scale
        for tau in torques_sum:
           
            # Find the corresponding max_value and min_value for the current tree_id
            max_value = next(item["max_value"] for item in max_min_value_torques if item["tree_id"] == tau["tree_id"])
            min_value = next(item["min_value"] for item in max_min_value_torques if item["tree_id"] == tau["tree_id"])
            
            norm_tau = (tau["sum"] - min_value ) / ( max_value - min_value)


            # append the normalized torque to the array
            norm_torques = np.append(norm_torques, {"norm_tau" : norm_tau, "end_effector_pos" : tau["end_effector_pos"], "tree_id" : tau["tree_id"]})

        return norm_torques
    

    def check_zero(self, vec : np.ndarray) -> bool:
        """
        Checks if the vector is zero.
        
        :param vec: Vector to check.
        :return: True if the acceleration vector is zero, False otherwise.
        """
        
        return np.allclose(vec, np.zeros(self.model.nv), atol=1e-6)


    def get_zero_configuration(self) -> np.ndarray:
        """
        Get the zero configuration of the robot model.
        
        :return: Zero configuration vector.
        """
        q0 = np.zeros(self.model.nq)
        if q0 is None:
            raise ValueError("Failed to get zero configuration")
        
        return q0
    

    def get_zero_velocity(self) -> np.ndarray:
        """
        Get the zero velocity vector of the robot model.
        
        :return: Zero velocity vector.
        """
        v0 = np.zeros(self.model.nv)
        if v0 is None:
            raise ValueError("Failed to get zero velocity")
        
        return v0
    

    def get_zero_acceleration(self) -> np.ndarray:
        """
        Get the zero acceleration vector of the robot model.
        
        :return: Zero acceleration vector.
        """
        a0 = np.zeros(self.model.nv)
        if a0 is None:
            raise ValueError("Failed to get zero acceleration")
        
        return a0
    

    def get_random_configuration(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get a random configuration for configuration and velocity vectors.
        :return: Random configuration vectors.
        """
        q_limits_lower = self.model.lowerPositionLimit
        q_limits_upper = self.model.upperPositionLimit
        
        # generate random configuration vector within the limits
        q = np.random.uniform(q_limits_lower, q_limits_upper)

        if q is None:
            raise ValueError("Failed to get random configuration")
        
        # Generate random velocity vector within the limits
        qdot = np.random.uniform(-self.model.velocityLimit, self.model.velocityLimit)
        if qdot is None:
            raise ValueError("Failed to get random velocity")
         
        return q, qdot
    

    def check_joint_limits(self, q : np.ndarray) -> np.ndarray:
        """
        Check if the joint configuration vector is within the joint limits of the robot model.
        
        :param q: Joint configuration vector to check.
        :return: Array of booleans indicating if each joint is within the limits.
        """
        if q is None:
            raise ValueError("Joint configuration vector is None")
        
        # array to store if the joint is within the limits
        within_limits = np.zeros(self.model.njoints - 1, dtype=bool)

        # Check if the joint configuration is within the limits
        for i in range(self.model.njoints - 1): 
            if q[i] < self.model.lowerPositionLimit[i] or q[i] > self.model.upperPositionLimit[i]:
                within_limits[i] = False
            else:
                within_limits[i] = True
        
        return within_limits


    def check_effort_limits(self, tau : np.ndarray, tree_id : int = None) -> np.ndarray:
        """
        Check if the torques vector is within the effort limits of the robot model.
        
        :param tau: Torques vector to check.
        :param tree_id: Id of tree to filter control in the torques vector.
        :return: Array of booleans indicating if each joint is within the effort limits.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        # array to store if the joint is within the limits
        within_limits = np.array([], dtype=bool)

        # if arm is not specified, check all joints
        if tree_id is None:
            # Check if the torques are within the limits
            for i in range(self.model.njoints -1): 
                if abs(tau[i]) > self.model.effortLimit[i]:
                    print(f"\033[91mJoint {i+2} exceeds effort limit: {tau[i]} > {self.model.effortLimit[i]} \033[0m\n")
                    within_limits = np.append(within_limits, False)
                else:
                    within_limits = np.append(within_limits, True)
            
            if np.all(within_limits):
                print("All joints are within effort limits. \n")
        
        else:
            # Check if the torque of joints inside the tree is within the limits
            for id in self.subtrees[tree_id]["joint_ids"]:
                if abs(tau[id-1]) > self.model.effortLimit[id-1]:
                    print(f"\033[91mJoint {id} exceeds effort limit: {tau[id-1]} > {self.model.effortLimit[id-1]} \033[0m\n")
                    within_limits = np.append(within_limits, False)
                else:
                    within_limits = np.append(within_limits, True)

            # for i in range(self.model.njoints -1):
            #     if arm in self.model.names[i+1]:
            #         if abs(tau[i]) > self.model.effortLimit[i]:
            #             print(f"\033[91mJoint {i+1} exceeds effort limit: {tau[i]} > {self.model.effortLimit[i]} \033[0m\n")
            #             within_limits = np.append(within_limits, False)
            #         else:
            #             within_limits = np.append(within_limits, True)
            
            if np.all(within_limits):
                print("All joints are within effort limits. \n")


        return within_limits


    def set_joint_tree_selection(self, tree_id : int, joint_id: int):
        """
        Set the joint tree selection for the robot model.
        
        :param tree_id: ID of the tree to select.
        :param joint_id: ID of the joint to select.
        """
        for tree in self.subtrees:
            if tree["tree_id"] == tree_id:
                # Set the selected joint in the tree
                tree["selected_joint_id"] = joint_id
                return

        

    def set_position(self, pos_joints : list[float], name_positions : list[str] ) -> np.ndarray:
        """
        Convert joint positions provided by jointstate publisher to the right format of the robot model.
        
        :param pos_joints: List of joint positions provided by jointstate publisher.
        :param name_positions: List of joint names in the order provided by jointstate publisher.
        """
        
        q = np.zeros(self.model.nq)
        cont = 0

        for i in range(1, np.size(pos_joints) + 1):
            # find index in name positions list that corrisponds to the joint name 
            # (REASON: the joint position from joint_state message are not in the same order as the joint indices in model object)
            index = name_positions.index(self.model.names[i]) 
            
            if self.model.joints[i].nq == 2:
                # for continuous joints (wheels)
                q[cont] = math.cos(pos_joints[index])
                q[cont + 1] = math.sin(pos_joints[index])
            else:
                # for revolute joints
                q[cont] = pos_joints[index]

            cont += self.model.joints[i].nq

        self.update_configuration(q)
        
        return q
    

    def get_position_for_joint_states(self, q : np.ndarray) -> np.ndarray:
        """
        Convert configuration in pinocchio format to right format of joint states publisher
        Example: continuous joints (wheels) are represented as two values in the configuration vector but 
        in the joint state publisher they are represented as one value (angle).

        :param q : Joint configuration provided by pinocchio library
        :return: Joint positions in the format of joint state publisher.
        """
        
        config = []
        
        current_selected_config = q

        # Use this method to get the single values for joints, for example continuous joints (wheels) are represented as two values in the configuration vector but 
        # in the joint state publisher they are represented as one value (angle)
        # so we need to convert the configuration vector to the right format for joint state publisher
        cont = 0
        for i in range(1, self.model.njoints):
            if self.model.joints[i].nq == 2:
                # for continuous joints (wheels)
                config.append({ "q" : math.atan2(current_selected_config[cont+1], current_selected_config[cont]), "joint_name" : self.model.names[i] })
                
            elif self.model.joints[i].nq == 1:
                # for revolute joints
                config.append({"q" : current_selected_config[cont], "joint_name" : self.model.names[i]})
            
            cont += self.model.joints[i].nq
        
        return config
    

    def get_joints_placements(self, q : np.ndarray) -> np.ndarray | float:
        """
        Get the placements of the joints in the robot model.
        
        :param q: Joint configuration vector.
        :return: Array of joint placements with names of joint, and z offset of base link.
        """
        base_link_id = self.model.getFrameId(self.root_name)
        offset_z = self.data.oMf[base_link_id].translation[2]  # Get the z offset of the base link

        self.update_configuration(q)
        placements = np.array([({"name" : self.model.names[i],
                                 "type" : self.model.joints[i].shortname() ,
                                 "id" : i,
                                 "x": self.data.oMi[i].translation[0], 
                                 "y": self.data.oMi[i].translation[1], 
                                 "z": self.data.oMi[i].translation[2]}) for i in range(1, self.model.njoints)], dtype=object)
        
        return placements, offset_z
    

    def get_joint_name(self, id_joint: int) -> np.ndarray:
        """
        Get the name of the joint by its ID.
        
        :param id_joint: ID of the joint to get the name for.
        :return: Name of the joint.
        """

        return self.model.names[id_joint]

    def get_joint_placement(self, joint_id : int, q : np.ndarray) -> dict:
        """
        Get the placement of a specific joint in the robot model.
        
        :param joint_id: ID of the joint to get the placement for.
        :param q: Joint configuration vector.
        :return: Dictionary with coordinates x , y , z of the joint.
        """
        
        self.update_configuration(q)

        if joint_id < 0 or joint_id >= self.model.njoints:
            raise ValueError(f"Joint ID {joint_id} is out of bounds for the robot model with {self.model.njoints} joints.")
        
        placement = self.data.oMi[joint_id].translation
        
        return {"x" : placement[0], "y": placement[1], "z": placement[2]}
        

    def get_mass_matrix(self, q : np.ndarray) -> np.ndarray:
        """
        Compute the mass matrix for the robot model.
        
        :param q: Joint configuration vector.
        :return: Mass matrix.
        """
        
        mass_matrix = pin.crba(self.model, self.data, q)
        if mass_matrix is None:
            raise ValueError("Failed to compute mass matrix")
        
        return mass_matrix
    

    def get_joints(self) -> np.ndarray:
        """
        Get the array joint names of the robot model.
        
        :return: array Joint names .
        """
        
        return np.array(self.model.names[1:], dtype=str)
    


    def get_frames(self) -> np.ndarray:
        """
        Get the array of frame names in the robot model.
        
        :return: array of frame names.
        """
        
        return np.array([frame.name for frame in self.model.frames if frame.type == pin.FrameType.BODY], dtype=str)
    


    def get_links(self,all_frames : bool = False) -> np.ndarray:
        """
        Get the array of link names for payload menu.
        :param all_frames: If True, return all frames, otherwise return only current tip frames.
        :return: array of link names.
        """
        # Get frames where joints are parents
        frame_names = []
        
        # If all_frames is True, get all link names from the subtrees
        if all_frames:
            for tree in self.subtrees:
                if tree["selected_joint_id"] is not None:
                    # insert links in the array
                    for link in tree["link_names"]:
                        frame_names.append(link)
        else:
            # if all_frames is False, get only the links connected to the selected joint IDs from the subtrees
            for tree in self.subtrees:
                if tree["selected_joint_id"] is not None:
                    link_name = self.get_links_from_tree(tree["selected_joint_id"])
                    frame_names.append(link_name[0])

        return np.array(frame_names, dtype=str)
    

    def get_root_name(self) -> str:
        """
        Get the name of the root frame of the robot model.
        
        :return: Name of the root frame.
        """
        
        if self.root_name is None:
            raise ValueError("Root name is not set")
        
        return self.root_name

    def get_parent_joint_id(self, frame_name : str) -> int:
        """
        Get the parent joint ID for a given frame name.
        
        :param frame_name: Name of the frame.
        :return: Joint ID.
        """
        
        if frame_name is None:
            raise ValueError("Frame name must be provided")
        
        # Get the frame ID from the model
        frame_id = self.model.getFrameId(frame_name)
        joint_id = self.model.frames[frame_id].parentJoint
        
        if joint_id == -1:
            raise ValueError(f"Joint '{joint_id}' not found in the robot model")
        
        return joint_id
    

    def print_configuration(self, q : np.ndarray = None):
        """
        Print the current configuration of the robot model.
        """

        self.update_configuration(q)

        for frame_id, frame in enumerate(self.model.frames):
            placement = self.data.oMf[frame_id]
            print(f"Frame: {frame.name}")
            print(f"  Rotation:\n{placement.rotation}")
            print(f"  Translation:\n{placement.translation}")


    def print_torques(self, tau : np.ndarray):
        """
        Print the torques vector.
        
        :param tau: Torques vector to print.
        """
        if tau is None:
            raise ValueError("Torques vector is None")

        print("Torques vector:")
        for i, torque in enumerate(tau):
            # check if the joint is a prismatic joint
            if self.model.joints[i+1].shortname() == "JointModelPZ":
                print(f"Joint {i+2} {self.model.names[i+1]}: {torque:.4f} N")
                
            # for revolute joints
            else:
                print(f"Joint {i+2} {self.model.names[i+1]}: {torque:.4f} Nm")
        print("\n")



    def print_frames(self):
        """
        Print the frames of the robot model.
        """
        print("Body frames available in the robot: ")
        for i, frame in enumerate(self.model.frames):
            if frame.type == pin.FrameType.BODY:
                print(f"ID: {i:2d} | Name: {frame.name:20s}")
        print("\n")


    def print_acceleration(self, qddot : np.ndarray):
        """
        Print the acceleration vector.
        
        :param a: Acceleration vector to print.
        """
        
        print("Acceleration vector:")
        for i, acc in enumerate(qddot):
            print(f"Joint {i+2} {self.model.names[i+1]}: {acc:.6f} rad/s^2")
        
        print("\n")


    def print_active_joint(self):
        """
        Print the active joints of the robot model.
        """
        if self.model is None:
            raise ValueError("Model is not initialized")
        
        print("Active joints:")
        for i in range(self.model.njoints):
            print(f"Joint {i+1}: {self.model.names[i]}")

        print("\n")
        # placement of each joint in the kinematic tree
        # for name, oMi in zip(self.model.names, self.data.oMi):
        #     print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))