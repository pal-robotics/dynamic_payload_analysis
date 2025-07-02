
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
from optik import Robot, SolverConfig
import tempfile
from ikpy import chain
import os




class TorqueCalculator:
    def __init__(self, robot_description : Union[str, Path]):
        """
        Initialize the Torques_calculator with the URDF model or XML format provided by robot_description topic.
        
        :param urdf_path: Path to the URDF file of the robot.
        :param robot_description: Robot description in XML format provided by /robot_description topic or path of URDF file.
        """

        # Load the robot model from path or XML string
        if isinstance(robot_description, str):
            self.model = pin.buildModelFromXML(robot_description)
            
            # TODO change parser in general for more unique solution
            
            # create temporary URDF file from the robot description string
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as temp_file:
                    temp_file.write(robot_description)
                    temp_urdf_path = temp_file.name

            self.geom_model = pin.buildGeomFromUrdf(self.model,temp_urdf_path,pin.GeometryType.COLLISION)
            #self.ik_model = chain.Chain.from_urdf_file(temp_urdf_path)
            #self.ik_model = Robot.from_urdf_file(temp_urdf_path, "base_link", "arm_left_7_link")
            os.unlink(temp_urdf_path)
            


        elif isinstance(robot_description, Path):
            self.model = pin.buildModelFromUrdf(str(robot_description.resolve()))
            
            # create data for IK solver
            self.ik_model = Robot.from_urdf_file(str(robot_description.resolve()), "base_link", "arm_left_7_link")
        
        # create data for the robot model
        self.ik_config = SolverConfig()
        
        self.data = self.model.createData()
        self.geom_data = pin.GeometryData(self.geom_model)

        
        

    def compute_inverse_dynamics(self, q : np.ndarray , qdot : np.ndarray, qddot : np.ndarray, extForce : np.ndarray[pin.Force] = None) -> np.ndarray:
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
    

    def create_ext_force(self, masses : Union[float, np.ndarray] , frame_name : Union[str | np.ndarray], q : np.ndarray) -> np.ndarray[pin.Force]:
        """
        Create external forces vector based on the mass and frame ID.
        The resulting vector will contain the force applied to the specified frame and with the local orientation of the parent joint.
        
        :param masses (float, np.ndarray) : Mass of the object to apply the force to or vector with masses related to frames names.
        :param frame_name(str , np.ndarray) : Frame name where the force is applied or vector of frame names where the forces is applied.
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
    


    def get_active_frames(self) -> np.ndarray:
        """
        Get the array of active joint names in the robot model.
        
        :return: array of active joint names.
        """
        # Get frames where joints are parents
        frame_names = []
        for i in range(1, self.model.njoints):
            for frame in self.model.frames:
                if frame.parentJoint == i and frame.type == pin.FrameType.BODY:
                    frame_names.append(frame.name)
                    break
        
        return np.array(frame_names, dtype=str)
    


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



    def compute_maximum_payload(self, q : np.ndarray, qdot : np.ndarray, tau : np.ndarray, frame_name : str) -> float:
        """
        Compute the forward dynamics acceleration vector.
        
        :param q: Joint configuration vector.
        :param qdot: Joint velocity vector.
        :param tau: Joint torque vector.
        :param frame_name: Name of the frame where the force is applied.
        :return: Acceleration vector.
        """
        
        # Update the configuration of the robot model
        self.update_configuration(q)

        # basic idea for forward dynamics: M(q) * a_q + b = tau(q) --> a_q = (tau(q) - b)/ M(q)
        # with external force, the equation becomes: M(q) * a_q + b = tau(q) + J_traspose(q) * f_ext -- > f_ext = (M(q) * a_q + b - tau(q))/ J_traspose(q)

        qddot0 = np.zeros(self.model.nv) # Initialize acceleration vector
        
        # calculate dynamics drift
        b = pin.rnea(self.model, self.data, q, qdot, qddot0)

        #get jacobian of the frame where the payload is applied
        J = self.compute_jacobian(q, frame_name)

        tau_r = b - tau
        
        try:
            #a = np.linalg.solve(M, tau - b)
            # get F = (J_transpose(q))^-1  X ( tau - b ) with b = M(q)*a_q + b || pinv = pseudo-inverse to prevent singularities
            F_max = np.linalg.pinv(J.T) @ tau_r

        except np.linalg.LinAlgError as e:
            raise ValueError(f"Failed to solve for acceleration: {e}")
        
        return F_max[2] # get the force in z axis of the world frame, which is the maximum force payload
    


    def compute_inverse_kinematics_optik(self, q : np.ndarray, end_effector_position: np.ndarray) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot model using the Optik library.
        
        :param q: current joint configuration vector.
        :param end_effector_position: Position of the end effector in the world frame [rotation matrix , translation vector].
        :return: Joint configuration vector that achieves the desired end effector position.
        """
        
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
        
        # Compute the inverse kinematics
        sol = self.ik_model.inverse_kinematics(end_effector_position)
        
        return sol



    def compute_inverse_kinematics(self, q : np.ndarray, end_effector_position: np.ndarray, end_effector_name : str) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot model with joint limits consideration.
        :param q: current joint configuration vector.
        :param end_effector_position: Position of the end effector in the world frame [rotation matrix , translation vector].
        :param end_effector_name: Name of the end effector joint.
        :return: Joint configuration vector that achieves the desired end effector position.
        """

        joint_id = self.model.getJointId(end_effector_name)  # Get the joint ID of the end effector

        # Set parameters for the inverse kinematics solver
        eps = 1e-3 # reduce for more precision
        IT_MAX = 500 # Maximum number of iterations
        DT = 1e-1 
        damp = 1e-12

        i = 0
        while True:
            self.update_configuration(q)
            iMd = self.data.oMi[joint_id].actInv(end_effector_position) # Get the transformation from the current end effector pose to the desired pose
            
            err = pin.log(iMd).vector  # compute the error in the end effector position
            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break

            J = pin.computeJointJacobian(self.model, self.data, q, joint_id)  # compute the Jacobian of current pose of end effector
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)

            # compute the inverse kinematics v = -J^T * (J * J^T + damp * I)^-1 * err
            v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
            
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
        
            

    def compute_all_configurations(self, range : int, resolution : int, end_effector_name : str) -> np.ndarray:
        """
        Compute all configurations for the robot model within a specified range.
        
        :param range (int): Range as side of a square where in the center there is the actual position of end effector.
        :param resolution (int): Resolution of the grid to compute configurations.
        :param end_effector_name (str): Name of the end effector joint.
        :return : Array of joint configurations that achieve the desired end effector position.
        """
        
        if range <= 0:
            raise ValueError("Range must be a positive value")
        
        # Get the current joint configuration
        q = self.get_zero_configuration()

        id_end_effector = self.model.getJointId(end_effector_name)
        # Get the current position of the end effector
        end_effector_pos = self.data.oMi[id_end_effector]
        
        # Create an array to store all configurations
        configurations = []
        
        # Iterate over the range to compute all configurations
        for x in np.arange(-range/2, range/2 , resolution):
            for y in np.arange(-range/2, range/2 , resolution):
                for z in np.arange(0, range , resolution):
                    target_position = pin.SE3(np.eye(3), np.array([x, y, z]))
                    new_q = self.compute_inverse_kinematics(q, target_position, end_effector_name)
                     
                    if new_q is not None:
                        q = new_q
                        # store the valid configuration and the position of the end effector relative to that configuration
                        configurations.append({"config" : new_q, "end_effector_pos": target_position.translation}) 
        
        return np.array(configurations, dtype=object)
    


    def verify_configurations(self, configurations : np.ndarray, masses : np.ndarray, checked_frames : np.ndarray) -> np.ndarray:
        """
        Verify the configurations to check if they are valid.
        
        :param configurations: Array of joint configurations to verify.
        :param ext_forces: Array of external forces to apply to the robot model.
        :return: Array of valid configurations with related torques in format: [{"config", "end_effector_pos, "tau"}].
        """
        
        valid_configurations = []
        
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
            if self.check_effort_limits(tau).all():

                # Compute all the collisions
                pin.computeCollisions(self.model, self.data, self.geom_model, self.geom_data, q["config"], False)

                # Print the status of collision for all collision pairs
                for k in range(len(self.geom_model.collisionPairs)):
                    cr = self.geom_data.collisionResults[k]
                    cp = self.geom_model.collisionPairs[k]
                    print(
                        "collision pair:",
                        cp.first,
                        ",",
                        cp.second,
                        "- collision:",
                        "Yes" if cr.isCollision() else "No",
                    )
                
                valid_configurations.append({"config" : q["config"], "end_effector_pos" : q["end_effector_pos"], "tau" : tau })
                
        return np.array(valid_configurations, dtype=object)
        

    def get_valid_workspace(self, range : int, resolution : int, end_effector_name : str, masses : np.ndarray, checked_frames: np.ndarray) -> np.ndarray:
        """
        Get the valid workspace of the robot model by computing all configurations within a specified range.
        
        :param range (int): Range as side of a square where in the center there is the actual position of end effector.
        :param resolution (int): Resolution of the grid to compute configurations.
        :param end_effector_name (str): Name of the end effector joint to test.
        :param ext_forces: Array of external forces to apply to the robot model.
        :return: Array of valid configurations that achieve the desired end effector position in format: [{"config", "end_effector_pos, "tau"}].
        """
        
        # Compute all configurations within the specified range
        configurations = self.compute_all_configurations(range,resolution, end_effector_name)
        
        # Verify the configurations to check if they are valid
        valid_configurations = self.verify_configurations(configurations, masses, checked_frames)
        
        return valid_configurations
    


    def compute_forward_dynamics_aba_method(self, q : np.ndarray, qdot : np.ndarray, tau : np.ndarray, extForce : np.ndarray[pin.Force] = None) -> np.ndarray:
        """
        Compute the forward dynamics acceleration vector with Articulated-Body algorithm(ABA).
        
        :param q: Joint configuration vector.
        :param qdot: Joint velocity vector.
        :param tau: Joint torque vector.
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
    

    def get_normalized_unified_torques(self, tau : np.ndarray) -> np.ndarray:
        """
        Normalize the torques vector to a unified scale.
        
        :param tau: Torques vector to normalize.
        :param arm: Arm name to filter the torques vector.
        :return: Normalized torques vector.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        norm_tau = []

        # Normalize the torques vector
        for i, torque in enumerate(tau):
            norm_tau.append(abs(torque) / self.model.effortLimit[i])
            
        
        return norm_tau
    

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
        :return: Random configuration vector.
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


    def check_effort_limits(self, tau : np.ndarray) -> np.ndarray:
        """
        Check if the torques vector is within the effort limits of the robot model.
        
        :param tau: Torques vector to check.
        :return: Array of booleans indicating if each joint is within the effort limits.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        # array to store if the joint is within the limits
        within_limits = np.zeros(self.model.njoints - 1, dtype=bool)

        # Check if the torques are within the limits
        for i in range(self.model.njoints -1): 
            if abs(tau[i]) > self.model.effortLimit[i]:
                print(f"\033[91mJoint {i+2} exceeds effort limit: {tau[i]} > {self.model.effortLimit[i]} \033[0m\n")
                within_limits[i] = False
            else:
                within_limits[i] = True
        
        if np.all(within_limits):
            print("All joints are within effort limits. \n")
        
        return within_limits



    def set_position(self, pos_joints : list[float], name_positions : list[str] ) -> np.ndarray:
        """
        Convert joint positions provided by jointstate publisher to the right format of the robot model.
        
        :param q: Joint configuration vector to set provided by joint state topic.
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

        :param q : Joint configuration provided by function get_valid_workspace() 
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
    

    def get_joints_placements(self, q : np.ndarray) -> np.ndarray | float:
        """
        Get the placements of the joints in the robot model.
        
        :param q: Joint configuration vector.
        :return: Array of joint placements with names of joint, and z offset of base link.
        """
        base_link_id = self.model.getFrameId("base_link")
        offset_z = self.data.oMf[base_link_id].translation[2]  # Get the z offset of the base link

        self.update_configuration(q)
        placements = np.array([({"name" : self.model.names[i],
                                 "type" : self.model.joints[i].shortname() , 
                                 "x": self.data.oMi[i].translation[0], 
                                 "y": self.data.oMi[i].translation[1], 
                                 "z": self.data.oMi[i].translation[2]}) for i in range(1, self.model.njoints)], dtype=object)
        
        return placements, offset_z
    

    def get_joint_placement(self, joint_id : int) -> dict:
        """
        Get the placement of a specific joint in the robot model.
        
        :param joint_id: ID of the joint to get the placement for.
        :return: Dictionary with coordinates x , y , z of the joint.
        """
        
        if joint_id < 0 or joint_id >= self.model.njoints:
            raise ValueError(f"Joint ID {joint_id} is out of bounds for the robot model with {self.model.njoints} joints.")
        
        placement = self.data.oMi[joint_id].translation
        
        return {"x" : placement[0], "y": placement[1], "z": placement[2]}
        


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