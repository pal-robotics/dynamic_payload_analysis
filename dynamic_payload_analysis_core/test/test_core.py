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

from dynamic_payload_analysis_core.core import TorqueCalculator
import unittest
import os
import numpy as np


class TestTorqueCalculator(unittest.TestCase):

    def __init__(self, methodName="runTest"):
        super().__init__(methodName)

        test_dir = os.path.dirname(__file__)
        urdf_path = os.path.join(test_dir, "test_xml.urdf")
        with open(urdf_path, "r") as f:
            self.xml_string = f.read()

        self.robot_handler = TorqueCalculator(
            robot_description=self.xml_string)

    def test_init_torque_calculator(self):
        self.assertIsNotNone(
            self.robot_handler.model,
            "Model should be initialized")
        self.assertIsNotNone(
            self.robot_handler.geom_model,
            "Geom model should be initialized")
        self.assertIsNotNone(
            self.robot_handler.geom_data,
            "Model data should be initialized")
        print("✅ Initialization assertions passed")

    def test_get_root_name(self):
        root_name = self.robot_handler.get_root_joint_name(self.xml_string)
        self.assertIsInstance(root_name, str, "Root name should be a string")
        self.assertEqual(self.robot_handler.get_root_name(), "base_footprint")
        self.assertEqual(root_name, self.robot_handler.get_root_name())
        print("✅ Root name assertion passed")

    def test_subtrees(self):
        subtrees = self.robot_handler.get_subtrees()

        self.assertIsNotNone(subtrees, "Subtrees should not be None")
        self.assertIsInstance(
            subtrees,
            np.ndarray,
            "Subtrees should be a list")
        self.assertEqual(len(subtrees), 7)
        self.assertEqual(subtrees[0]["tip_link_name"], "wheel_front_left_link")
        self.assertEqual(
            subtrees[1]["tip_link_name"],
            "wheel_front_right_link")
        self.assertEqual(subtrees[2]["tip_link_name"], "wheel_rear_left_link")
        self.assertEqual(subtrees[3]["tip_link_name"], "wheel_rear_right_link")
        self.assertEqual(
            subtrees[4]["tip_link_name"],
            "gripper_left_outer_finger_left_link")
        self.assertEqual(
            subtrees[5]["tip_link_name"],
            "gripper_right_outer_finger_left_link")
        self.assertEqual(subtrees[6]["tip_link_name"], "head_2_link")
        print("✅ Subtrees assertion passed")

    def test_compute_mimic_joints(self):
        mimic_joints = self.robot_handler.mimic_joint_names
        self.assertEqual(len(mimic_joints), 10)
        self.assertIn("gripper_left_inner_finger_left_joint", mimic_joints)
        self.assertIn("gripper_right_inner_finger_left_joint", mimic_joints)
        self.assertNotIn("arm_1_joint", mimic_joints)
        print("✅ Mimic joints assertion passed")

    def test_create_external_force(self):
        ext_forces = self.robot_handler.create_ext_force(
            masses=1,
            frame_name="arm_left_3_link",
            q=self.robot_handler.get_zero_configuration())
        self.assertIsInstance(
            ext_forces,
            list,
            "External forces should be a numpy array")
        self.assertNotEqual(
            ext_forces[8], [
                0, 0, 0], "External force should not be zero")
        print("✅ External force assertion passed")

    def test_compute_inverse_dynamics(self):
        standard_config = self.robot_handler.get_zero_configuration()

        torques = self.robot_handler.compute_inverse_dynamics(
            standard_config,
            self.robot_handler.get_zero_velocity(),
            self.robot_handler.get_zero_acceleration())

        self.assertIsNotNone(torques, "Torques should not be None")
        self.assertIsInstance(torques, np.ndarray, "Torques should be a list")
        self.assertTrue(all(isinstance(torque, float)
                        for torque in torques), "All torques should be floats")

        # add external forces to the end-effector
        external_forces = self.robot_handler.create_ext_force(
            masses=1, frame_name="arm_left_3_link", q=standard_config)
        torques_with_ext = self.robot_handler.compute_inverse_dynamics(
            standard_config,
            self.robot_handler.get_zero_velocity(),
            self.robot_handler.get_zero_acceleration(),
            external_forces)

        np.testing.assert_raises(
            AssertionError,
            np.testing.assert_array_equal,
            torques,
            torques_with_ext)
        print("✅ Inverse dynamics assertion passed")

    def test_update_configuration(self):
        zero_config = self.robot_handler.get_zero_configuration()
        new_config = zero_config + 5

        self.robot_handler.update_configuration(new_config)
        # just to ensure the configuration is updated internally
        joint_placement = self.robot_handler.get_joint_placement(
            joint_id=20, q=new_config)

        self.assertIsInstance(
            joint_placement,
            dict,
            "Joint placement should be a dict")

        self.robot_handler.update_configuration(zero_config)
        updated_joint_placement = self.robot_handler.get_joint_placement(
            joint_id=20, q=zero_config)

        self.assertNotEqual(
            joint_placement["x"],
            updated_joint_placement["x"],
            "Joint placement should differ after configuration update")
        self.assertNotEqual(
            joint_placement["y"],
            updated_joint_placement["y"],
            "Joint placement should differ after configuration update")
        self.assertNotEqual(
            joint_placement["z"],
            updated_joint_placement["z"],
            "Joint placement should differ after configuration update")

        print("✅ Update configuration assertion passed")

    def test_compute_inverse_kinematics(self):
        initial_config = self.robot_handler.get_zero_configuration()

        # reachable position
        end_effector_pos = self.robot_handler.get_end_effector_position_array(
            -0.4, -0.6, 0.6)
        computed_config = self.robot_handler.compute_inverse_kinematics(
            q=initial_config, end_effector_position=end_effector_pos, joint_id=25)

        self.assertIsNotNone(
            computed_config,
            "Computed configuration should not be None if IK is successful")
        self.assertIsInstance(
            computed_config,
            np.ndarray,
            "Computed configuration should be a numpy array")

        end_effector_pos_not_reachable = self.robot_handler.get_end_effector_position_array(
            4, 6, 6)  # not reachable position
        computed_config_not_reachable = self.robot_handler.compute_inverse_kinematics(
            q=initial_config, end_effector_position=end_effector_pos_not_reachable, joint_id=25)

        self.assertIsNone(
            computed_config_not_reachable,
            "Computed configuration should be None if IK is not successful")

        print("✅ Inverse kinematics assertion passed")

    def test_compute_valid_workspace(self):
        computed_configs = self.robot_handler.compute_all_configurations(
            range=1,
            resolution=0.3,
            end_joint_id=25)  # used 1 meter range and 0.3 resolution to speed up the test
        len_configs = len(computed_configs)
        self.assertIsInstance(
            computed_configs,
            np.ndarray,
            "Computed configuration should be a numpy array")

        config_number = round(len_configs / 2)
        self.assertTrue(np.all(computed_configs[config_number]["end_effector_pos"] <= np.array(
            [1, 1, 1])), "End effector positions should be within the specified range")
        config_number = round(len_configs / 3)
        self.assertTrue(np.all(computed_configs[config_number]["end_effector_pos"] <= np.array(
            [1, 1, 1])), "End effector positions should be within the specified range")

        verified_configs = self.robot_handler.verify_configurations(
            computed_configs,
            masses=None,
            checked_frames=None,
            tree_id=5,
            selected_joint_id=25)

        self.assertIsInstance(
            verified_configs,
            np.ndarray,
            "Verified configuration should be a numpy array")

        self.assertEqual(
            verified_configs[0]["tree_id"],
            5,
            "Tree ID should match the provided tree ID")
        self.assertEqual(
            verified_configs[0]["selected_joint_id"],
            25,
            "Selected joint ID should match the provided joint ID")

        # should be None because there is no selected joint id inside the
        # object
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        self.assertIsInstance(
            valid_configurations,
            np.ndarray,
            "Valid workspace should be a numpy array")
        self.assertEqual(
            len(valid_configurations),
            0,
            "Valid workspace length should be zero if there is no selected joint id")

        # modify selected joint id to test valid workspace
        self.robot_handler.subtrees[5]["selected_joint_id"] = 25
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        self.assertEqual(
            len(valid_configurations),
            len(verified_configs),
            "Valid workspace length should match the verified configurations length")

        print("✅ Compute valid workspace assertion passed")

    def test_compute_maximum_payloads(self):
        self.robot_handler.subtrees[5]["selected_joint_id"] = 25
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        max_payload = self.robot_handler.find_max_payload_binary_search(
            config=valid_configurations[0], payload_min=0.0, payload_max=10.0, resolution=0.01)

        self.assertIsInstance(
            max_payload,
            float,
            "Max payload should be a float")
        self.assertGreaterEqual(
            max_payload, 0.0, "Max payload should be non-negative")
        self.assertLessEqual(
            max_payload,
            10.0,
            "Max payload should be within the specified range")

        max_payload = self.robot_handler.find_max_payload_binary_search(
            config=valid_configurations[0], payload_min=0.0, payload_max=0, resolution=0.01)
        self.assertEqual(
            max_payload,
            0.0,
            "Max payload should be zero if payload_max is zero")

        configs_with_payloads = self.robot_handler.compute_maximum_payloads(
            valid_configurations)

        self.assertIsInstance(
            configs_with_payloads,
            np.ndarray,
            "Configurations with payloads should be a numpy array")
        self.assertIn(
            "max_payload",
            configs_with_payloads[0],
            "Each configuration should have a max_payload key")
        self.assertIsInstance(
            configs_with_payloads[0]["max_payload"],
            float,
            "Max payload should be a float")

        print("✅ Compute maximum payloads assertion passed")

    def test_compute_forward_dynamics_aba_method(self):
        zero_config = self.robot_handler.get_zero_configuration()
        zero_velocity = self.robot_handler.get_zero_velocity()
        tau = np.zeros(self.robot_handler.model.nv)

        computed_acceleration = self.robot_handler.compute_forward_dynamics_aba_method(
            q=zero_config, qdot=zero_velocity, tau=tau)

        self.assertIsNotNone(
            computed_acceleration,
            "Computed acceleration should not be None")
        self.assertIsInstance(
            computed_acceleration,
            np.ndarray,
            "Computed acceleration should be a numpy array")

        print("✅ Compute forward dynamics ABA method assertion passed")

    def test_compute_jacobian(self):
        zero_config = self.robot_handler.get_zero_configuration()
        jacobian = self.robot_handler.compute_jacobian(
            q=zero_config, frame_name="arm_left_3_link")

        self.assertIsNotNone(jacobian, "Jacobian should not be None")
        self.assertIsInstance(
            jacobian,
            np.ndarray,
            "Jacobian should be a numpy array")
        self.assertEqual(
            jacobian.shape,
            (6,
             self.robot_handler.model.nv),
            "Jacobian shape should be (6, nv)")

        print("✅ Compute Jacobian assertion passed")

    def test_verify_member_tree(self):
        flag = self.robot_handler.verify_member_tree(tree_id=5, joint_id=25)

        self.assertTrue(flag, "Joint ID should be in the specified tree ID")
        self.assertIsInstance(flag, bool, "Flag should be a boolean")

        flag = self.robot_handler.verify_member_tree(tree_id=5, joint_id=2)

        self.assertFalse(
            flag, "Joint ID should not be in the specified tree ID")

        print("✅ Verify member tree assertion passed")

    def test_get_links_from_tree(self):
        link = self.robot_handler.get_links_from_tree(joint_ids=25)
        links = self.robot_handler.get_links_from_tree(joint_ids=[25, 26, 24])

        self.assertIsInstance(
            links,
            np.ndarray,
            "Links should be a numpy array")
        self.assertEqual(
            len(links),
            3,
            "There should be 3 links in the subtree")

        self.assertIn(
            "arm_right_7_link",
            link,
            "Link should be in the subtree")
        self.assertIsNotNone(link, "Link should not be None")
        self.assertIsInstance(link, np.ndarray, "Link should be a numpy array")

        print("✅ Get links from tree assertion passed")

    def test_get_maximum_torques(self):
        # modify selected joint id to test valid workspace
        self.robot_handler.subtrees[5]["selected_joint_id"] = 25
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        max_torques = self.robot_handler.get_maximum_torques(
            valid_configurations)

        self.assertIsNotNone(max_torques, "Max torques should not be None")
        self.assertIsInstance(
            max_torques,
            np.ndarray,
            "Max torques should be a numpy array")

        # get selected subtrees only
        selected_subtrees = [subtree for subtree in self.robot_handler.get_subtrees(
        ) if subtree["selected_joint_id"] is not None]

        self.assertEqual(
            len(max_torques),
            len(selected_subtrees),
            "Max torques length should match the number of selected subtrees")
        self.assertIn(
            "max_values",
            max_torques[0],
            "Each max torque entry should have a max_values key")
        self.assertEqual(
            max_torques[0]["tree_id"],
            selected_subtrees[0]["tree_id"],
            "Tree ID should match the selected subtree's tree ID")

        print("✅ Get maximum torques assertion passed")

    def test_get_maximum_payloads(self):
        # modify selected joint id to test valid workspace
        self.robot_handler.subtrees[5]["selected_joint_id"] = 25
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)
        valid_configurations = self.robot_handler.compute_maximum_payloads(
            valid_configurations)

        max_payloads = self.robot_handler.get_maximum_payloads(
            valid_configurations)

        self.assertIsNotNone(max_payloads, "Max payloads should not be None")
        self.assertIsInstance(
            max_payloads,
            np.ndarray,
            "Max payloads should be a numpy array")
        self.assertGreater(
            len(max_payloads),
            0,
            "Max payloads length should be greater than zero")
        self.assertIn(
            "max_payload",
            max_payloads[0],
            "Each max payload entry should have a max_payload key")
        self.assertIsInstance(
            max_payloads[0]["max_payload"],
            float,
            "Max payload values should be a float")

        # get selected subtrees only
        selected_subtrees = [subtree for subtree in self.robot_handler.get_subtrees(
        ) if subtree["selected_joint_id"] is not None]

        self.assertEqual(
            len(max_payloads),
            len(selected_subtrees),
            "Max payloads length should match the number of selected subtrees")
        self.assertEqual(
            max_payloads[0]["tree_id"],
            selected_subtrees[0]["tree_id"],
            "Tree ID should match the selected subtree's tree ID")

        print("✅ Get maximum payloads assertion passed")

    def test_get_normalized_torques(self):
        standard_config = self.robot_handler.get_zero_configuration()
        # get torques for standard configuration
        torques = self.robot_handler.compute_inverse_dynamics(
            standard_config,
            self.robot_handler.get_zero_velocity(),
            self.robot_handler.get_zero_acceleration())

        normalized_torques = self.robot_handler.get_normalized_torques(torques)

        self.assertIsNotNone(
            normalized_torques,
            "Normalized torques should not be None")
        self.assertIsInstance(
            normalized_torques,
            np.ndarray,
            "Normalized torques should be a numpy array")
        self.assertEqual(
            len(normalized_torques),
            len(torques),
            "Normalized torques length should match the torques length")
        self.assertTrue(all(0 <= nt <= 1.0 for nt in normalized_torques),
                        "All normalized torques should be between 0 and 1")

        self.robot_handler.set_joint_tree_selection(tree_id=5, joint_id=25)
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        max_torques = self.robot_handler.get_maximum_torques(
            valid_configurations)
        normalized_torques_with_max = self.robot_handler.get_normalized_torques(
            torques, target_torque=max_torques, tree_id=5)

        self.assertIsNotNone(normalized_torques_with_max,
                             "Normalized torques with max should not be None")
        self.assertIsInstance(
            normalized_torques_with_max,
            np.ndarray,
            "Normalized torques with max should be a numpy array")
        self.assertEqual(
            len(normalized_torques_with_max),
            len(torques),
            "Normalized torques with max length should match the torques length")

        print("✅ Get normalized torques assertion passed")

    def test_get_normalized_payload(self):
        norm_payload = self.robot_handler.get_normalized_payload(
            payload=5, target_payload=10)

        self.assertIsNotNone(norm_payload,
                             "Normalized payload should not be None")
        self.assertIsInstance(
            norm_payload,
            float,
            "Normalized payload should be a float")
        self.assertGreaterEqual(
            norm_payload,
            0.0,
            "Normalized payload should be non-negative")
        self.assertEqual(
            norm_payload,
            0.5,
            "Normalized payload should be 0.5 for payload 5 and target payload 10")

        print("✅ Get normalized payload assertion passed")

    def test_get_unified_configurations_torque(self):
        self.robot_handler.set_joint_tree_selection(tree_id=5, joint_id=25)
        valid_configurations = self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)
        valid_configurations = self.robot_handler.compute_maximum_payloads(
            valid_configurations)

        unified_configs = self.robot_handler.get_unified_configurations_torque(
            valid_configurations)

        self.assertIsNotNone(unified_configs,
                             "Unified configurations should not be None")
        self.assertIsInstance(
            unified_configs,
            np.ndarray,
            "Unified configurations should be a numpy array")
        self.assertIn(
            "norm_tau",
            unified_configs[0],
            "Each unified configuration should have a normalized_torque key")
        self.assertEqual(
            len(unified_configs),
            len(valid_configurations),
            "Unified configurations length should match the valid configurations length")

        print("✅ Get unified configurations torque assertion passed")

    def test_check_zero(self):
        zero_config = np.zeros(self.robot_handler.model.nv)
        non_zero_config = zero_config + 1

        self.assertTrue(
            self.robot_handler.check_zero(zero_config),
            "Zero configuration should be identified as zero")
        self.assertFalse(
            self.robot_handler.check_zero(non_zero_config),
            "Non-zero configuration should not be identified as zero")

        print("✅ Check zero assertion passed")

    def test_get_zero_config_vel_acc(self):
        zero_config = self.robot_handler.get_zero_configuration()
        zero_velocity = self.robot_handler.get_zero_velocity()
        zero_acceleration = self.robot_handler.get_zero_acceleration()

        self.assertIsNotNone(zero_config,
                             "Zero configuration should not be None")
        self.assertIsNotNone(zero_velocity, "Zero velocity should not be None")
        self.assertIsNotNone(
            zero_acceleration,
            "Zero acceleration should not be None")

        self.assertIsInstance(
            zero_config,
            np.ndarray,
            "Zero configuration should be a numpy array")
        self.assertIsInstance(
            zero_velocity,
            np.ndarray,
            "Zero velocity should be a numpy array")
        self.assertIsInstance(
            zero_acceleration,
            np.ndarray,
            "Zero acceleration should be a numpy array")

        self.assertTrue(all(v == 0.0 for v in zero_config),
                        "All elements in zero configuration should be zero")
        self.assertTrue(all(v == 0.0 for v in zero_velocity),
                        "All elements in zero velocity should be zero")
        self.assertTrue(all(v == 0.0 for v in zero_acceleration),
                        "All elements in zero acceleration should be zero")

        print("✅ Get zero configuration, velocity, and acceleration assertion passed")

    def test_get_analyzed_points(self):
        analyzed_points = self.robot_handler.get_analyzed_points()

        self.assertIsInstance(
            analyzed_points,
            np.ndarray,
            "Analyzed points should be a numpy array")
        self.assertEqual(
            len(analyzed_points),
            0,
            "Analyzed points length should be zero initially")

        self.robot_handler.set_joint_tree_selection(tree_id=5, joint_id=25)
        self.robot_handler.get_valid_workspace(
            range=1, resolution=0.3, masses=None, checked_frames=None)

        analyzed_points = self.robot_handler.get_analyzed_points()

        self.assertGreater(
            len(analyzed_points),
            0,
            "Analyzed points length should be greater than zero after computing valid workspace")

        print("✅ Get analyzed points assertion passed")

    def test_get_random_configuration(self):
        q, qdot = self.robot_handler.get_random_configuration()

        self.assertIsNotNone(q, "Random configuration should not be None")
        self.assertIsInstance(
            q, np.ndarray, "Random configuration should be a numpy array")
        self.assertIsNotNone(qdot, "Random velocity should not be None")
        self.assertIsInstance(
            qdot,
            np.ndarray,
            "Random velocity should be a numpy array")

        print("✅ Get random configuration assertion passed")

    def test_check_joint_limits(self):
        within_limits_config = self.robot_handler.get_zero_configuration()
        out_of_limits_config = within_limits_config + \
            100  # Assuming this will exceed joint limits

        self.assertIsInstance(
            self.robot_handler.check_joint_limits(within_limits_config),
            np.ndarray,
            "Return value should be a numpy array of booleans")
        self.assertTrue(
            np.all(
                self.robot_handler.check_joint_limits(within_limits_config)),
            "Configuration within limits should return True")
        self.assertFalse(
            np.all(
                self.robot_handler.check_joint_limits(out_of_limits_config)),
            "Configuration out of limits should return False")

        print("✅ Check joint limits assertion passed")

    def test_check_effort_limits(self):
        low_tau = self.robot_handler.compute_inverse_dynamics(
            self.robot_handler.get_zero_configuration(),
            self.robot_handler.get_zero_velocity(),
            self.robot_handler.get_zero_acceleration())
        high_tau = low_tau + 100

        self.assertIsInstance(
            self.robot_handler.check_effort_limits(
                low_tau,
                tree_id=5),
            np.ndarray,
            "Return value should be a numpy array of booleans")
        self.assertTrue(
            np.all(
                self.robot_handler.check_effort_limits(
                    low_tau,
                    tree_id=5)),
            "Torques within limits should return True")
        self.assertFalse(
            np.all(
                self.robot_handler.check_effort_limits(
                    high_tau,
                    tree_id=5)),
            "Torques out of limits should return False")

        print("✅ Check effort limits assertion passed")

    def test_get_position_for_joint_states(self):
        q = self.robot_handler.get_zero_configuration()

        position = self.robot_handler.get_position_for_joint_states(q)

        self.assertIsNotNone(position, "Position should not be None")
        self.assertIsInstance(
            position,
            np.ndarray,
            "Position should be a numpy array")

        print("✅ Get position for joint states assertion passed")

    def test_get_joints_placements(self):
        q = self.robot_handler.get_zero_configuration()

        joint_placement, offset = self.robot_handler.get_joints_placements(q=q)

        self.assertIsNotNone(
            joint_placement,
            "Joint placement should not be None")
        self.assertIsInstance(
            joint_placement,
            np.ndarray,
            "Joint placement should be a numpy array")
        self.assertIsNotNone(offset, "Offset should not be None")
        self.assertIsInstance(offset, float, "Offset should be a float")

    def test_get_joint_placement(self):
        q = self.robot_handler.get_zero_configuration()

        joint_placement = self.robot_handler.get_joint_placement(
            joint_id=20, q=q)

        self.assertIsNotNone(
            joint_placement,
            "Joint placement should not be None")
        self.assertIsInstance(
            joint_placement,
            dict,
            "Joint placement should be a dictionary")
        self.assertIn(
            "x",
            joint_placement,
            "Joint placement should have an 'x' key")
        self.assertIn(
            "y",
            joint_placement,
            "Joint placement should have a 'y' key")
        self.assertIn(
            "z",
            joint_placement,
            "Joint placement should have a 'z' key")

        print("✅ Get joint placement assertion passed")

    def test_get_joint_name(self):
        joint_name = self.robot_handler.get_joint_name(joint_id=25)

        self.assertIsNotNone(joint_name, "Joint name should not be None")
        self.assertIsInstance(joint_name, str, "Joint name should be a string")
        self.assertEqual(
            joint_name,
            "arm_right_7_joint",
            "Joint name should match the expected value")

        print("✅ Get joint name assertion passed")

    def test_get_mass_matrix(self):
        q = self.robot_handler.get_zero_configuration()

        mass_matrix = self.robot_handler.get_mass_matrix(q=q)

        self.assertIsNotNone(mass_matrix, "Mass matrix should not be None")
        self.assertIsInstance(
            mass_matrix,
            np.ndarray,
            "Mass matrix should be a numpy array")
        self.assertEqual(
            mass_matrix.shape,
            (self.robot_handler.model.nv,
             self.robot_handler.model.nv),
            "Mass matrix shape should be (nv, nv)")

        print("✅ Get mass matrix assertion passed")

    def test_get_joints(self):
        joints = self.robot_handler.get_joints()

        self.assertIsNotNone(joints, "Joints should not be None")
        self.assertIsInstance(joints, np.ndarray, "Joints should be a list")
        self.assertGreater(len(joints), 0, "Joints list should not be empty")
        self.assertIn(
            "arm_right_1_joint",
            joints,
            "Joints list should contain 'arm_right_1_joint'")

        print("✅ Get joints assertion passed")

    def test_get_frames(self):
        frames = self.robot_handler.get_frames()

        self.assertIsNotNone(frames, "Frames should not be None")
        self.assertIsInstance(frames, np.ndarray, "Frames should be a list")
        self.assertGreater(len(frames), 0, "Frames list should not be empty")
        self.assertIn(
            "arm_right_1_link",
            frames,
            "Frames list should contain 'arm_right_1_link'")

        print("✅ Get frames assertion passed")

    def test_get_links(self):
        self.robot_handler.set_joint_tree_selection(tree_id=5, joint_id=25)
        links = self.robot_handler.get_links(all_frames=True)

        self.assertIsNotNone(links, "Links should not be None")
        self.assertIsInstance(links, np.ndarray, "Links should be a list")
        self.assertGreater(len(links), 0, "Links list should not be empty")
        self.assertIn(
            "arm_right_1_link",
            links,
            "Links list should contain 'arm_right_1_link'")

        only_tip_links = self.robot_handler.get_links(all_frames=False)

        self.assertIsNotNone(only_tip_links, "Links should not be None")
        self.assertIsInstance(
            only_tip_links,
            np.ndarray,
            "Links should be a list")
        self.assertGreater(
            len(only_tip_links),
            0,
            "Links list should not be empty")
        self.assertLess(
            len(only_tip_links),
            len(links),
            "Tip links list should be smaller than all links list")
        self.assertIn(
            "arm_right_7_link",
            only_tip_links,
            "Tip links list should contain 'arm_right_7_link'")

        print("✅ Get links assertion passed")

    def test_get_parent_joint_id(self):
        parent_id = self.robot_handler.get_parent_joint_id(
            frame_name="arm_right_7_link")

        self.assertIsNotNone(parent_id, "Parent joint ID should not be None")
        self.assertIsInstance(
            parent_id, int, "Parent joint ID should be an integer")
        self.assertEqual(
            parent_id,
            25,
            "Parent joint ID should match the expected value")

        print("✅ Get parent joint ID assertion passed")


if __name__ == "__main__":
    unittest.main()
