# python 3.10.12 /bin/python3

from core import Torques_calculator
import os

# good example of forward and inverse dynamics --> static-contact-dynamics.py on examples page

my_robot = Torques_calculator(urdf_path=os.path.join(os.path.dirname(__file__), "../URDF_folder", "my_urdf.urdf"))

my_robot.print_activejoint()

q, v, a = my_robot.get_zero_configuration(), my_robot.get_zero_velocity(), my_robot.get_zero_acceleration()


# torques without external force
#external_force = my_robot.create_ext_force(mass=1.0, frame_name="arm_7_link")

tau = my_robot.compute_inverse_dy(q, v, a, extForce=None)
my_robot.print_torques(tau)

#my_robot.print_frames()


# # # torques with external force with mass 10 kg
external_force = my_robot.create_ext_force(mass=.0, frame_name="arm_7_link")
#print(external_force)
tau = my_robot.compute_inverse_dy(q, v, a, extForce=external_force)
my_robot.check_effort_limits(tau)
my_robot.print_torques(tau)

# print(my_robot.get_Jacobian(q,"arm_7_link"))

#tau[4] = 41.0  # Set the torque for the arm_7_link to zero to check if the robot can hold the payload

force = my_robot.compute_maximum_payload(q, v, tau, frame_name="arm_7_link")

# print the maximum force that the robot can hold
print(f"Maximum force: {force} N  that corrisponds to a payload of {force / 9.81} kg \n")


# get the acceleration of the robot using ABA method
acc_aba = my_robot.compute_forward_dy_aba_method(q, v, tau, extForce=external_force)


if my_robot.check_zero_acceleration(acc_aba):
    print("Zero acceleration achieved.\n")
else:
    print("Non-zero acceleration achieved. ERROR the robot can't hold the current payload. \n")


my_robot.print_acceleration(acc_aba)