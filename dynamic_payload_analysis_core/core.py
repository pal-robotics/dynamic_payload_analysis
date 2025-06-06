# Library to handle calculations for inverse and forward dynamics

import pinocchio as pin
import numpy as np
import math
from typing import Union
from pathlib import Path

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
        elif isinstance(robot_description, Path):
            self.model = pin.buildModelFromUrdf(str(robot_description.resolve()))
        
        self.data = self.model.createData()
        

    def compute_inverse_dynamics(self, q, qdot, qddot, extForce : list[pin.Force] = None) -> list:
        """
        Compute the inverse dynamics torque vector.
        
        :param q: Joint configuration vector.
        :param qdot: Joint velocity vector.
        :param a: Joint acceleration vector.
        :return: Torques vector.
        """

        # basic equation for inverse dynamics : M(q) *a + b = tau + J(q)_t * extForce --> tau = M(q) * a + b - J(q)_t * extForce

        if extForce:
            tau = pin.rnea(self.model, self.data, q, qdot, qddot, extForce)
        else:
            tau = pin.rnea(self.model, self.data, q, qdot, qddot)
        
        if tau is None:
            raise ValueError("Failed to compute torques")
        
        return tau
    

    def create_ext_force(self, mass : float, frame_name, q) -> list[pin.Force]:
        """
        Create an external force vector based on the mass and frame ID.
        
        :param mass: Mass of the object to apply the force to.
        :param frame_id: Frame ID where the force is applied.
        :return: External force vector.
        """
        if mass < 0:
            raise ValueError("Mass must be a positive value")
        
        if frame_name is None:
            raise ValueError("Frame name must be provided")
        
        #if frame_name not in self.model.frames:
        #    raise ValueError(f"Frame name '{frame_name}' not found in the robot model")

        # assumption made : the force is applied to the joint
        
        # Initialize external forces array
        fext = [pin.Force(np.zeros(6)) for _ in range(self.model.njoints)]
        
        self.update_configuration(q) 

        # Get the frame ID and joint ID from the frame name
        frame_id = self.model.getFrameId(frame_name)
        joint_id = self.model.frames[frame_id].parentJoint

        # force in the world frame
        ext_force_world = pin.Force(np.array([0.0, 0.0, mass * -9.81]), np.array([0.0, 0.0, 0.0]))  # force in z axis of the world frame

        # placement of the frame in the world frame
        frame_placement = self.data.oMf[frame_id]
        #print(f"Frame placement: {frame_placement}")
        
        # Apply the force to the joint in the world frame
        fext[joint_id] = self.data.oMi[joint_id].actInv(ext_force_world)
        # Zero out the last 3 components (torques) of the force to ensure only the force in z axis (gravity force) is applied
        fext[joint_id].angular = np.zeros(3) # TODO : make it more efficient 
        
        return fext


    def update_configuration(self, q):
        """
        Update the robot model configuration with the given joint configuration vector.
        
        :param q: Joint configuration vector.
        """
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)


    def get_mass_matrix(self, q) -> np.ndarray:
        """
        Compute the mass matrix for the robot model.
        
        :param q: Joint configuration vector.
        :return: Mass matrix.
        """
        
        M = pin.crba(self.model, self.data, q)
        if M is None:
            raise ValueError("Failed to compute mass matrix")
        
        return M

    def compute_maximum_payload(self, q, qdot, tau, frame_name) -> float:
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
    

    def compute_forward_dy_aba_method(self, q, qdot, tau, extForce : list[pin.Force] = None) -> list:
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
    

    def compute_jacobian(self, q, frame_name) -> list:
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
    

    def check_zero(self, vec : list) -> bool:
        """
        Checks if the vector is zero.
        
        :param vec: Vector to check.
        :return: True if the acceleration vector is zero, False otherwise.
        """
        
        return np.allclose(vec, np.zeros(self.model.nv), atol=1e-6)


    def get_zero_configuration(self) -> list:
        """
        Get the zero configuration of the robot model.
        
        :return: Zero configuration vector.
        """
        q0 = np.zeros(self.model.nq)
        if q0 is None:
            raise ValueError("Failed to get zero configuration")
        
        return q0
    

    def get_zero_velocity(self) -> list:
        """
        Get the zero velocity vector of the robot model.
        
        :return: Zero velocity vector.
        """
        v0 = np.zeros(self.model.nv)
        if v0 is None:
            raise ValueError("Failed to get zero velocity")
        
        return v0
    

    def get_zero_acceleration(self) -> list:
        """
        Get the zero acceleration vector of the robot model.
        
        :return: Zero acceleration vector.
        """
        a0 = np.zeros(self.model.nv)
        if a0 is None:
            raise ValueError("Failed to get zero acceleration")
        
        return a0
    

    def get_random_configuration(self) -> tuple[list, list]:
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


    def check_effort_limits(self, tau) -> bool:
        """
        Check if the torques vector is within the effort limits of the robot model.
        
        :param tau: Torques vector to check.
        :return: True if within limits, False otherwise.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        # Check if the torques are within the limits
        for i in range(self.model.njoints -1): 
            if abs(tau[i]) > self.model.effortLimit[i]:
                print(f"\033[91mJoint {i+2} exceeds effort limit: {tau[i]} > {self.model.effortLimit[i]} \033[0m\n")
                return False
        
        print("All joints are within effort limits. \n")
        return True



    def set_position(self, pos_joints : list[float], name_positions : list[str] ) -> np.ndarray:
        """
        Set the joint positions of the robot model.
        
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

    def print_configuration(self):
        """
        Print the current configuration of the robot model.
        """
        for frame_id, frame in enumerate(self.model.frames):
            placement = self.data.oMf[frame_id]
            print(f"Frame: {frame.name}")
            print(f"  Rotation:\n{placement.rotation}")
            print(f"  Translation:\n{placement.translation}")


    def print_torques(self, tau):
        """
        Print the torques vector.
        
        :param tau: Torques vector to print.
        """
        if tau is None:
            raise ValueError("Torques vector is None")
        
        print("Torques vector:")
        for i, torque in enumerate(tau):
            # TODO Extract type of joint in order to print right measure unit ([N] or [Nm])
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


    def print_acceleration(self, qddot):
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