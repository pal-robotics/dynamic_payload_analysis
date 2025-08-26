from dynamic_payload_analysis_core.core import TorqueCalculator
import unittest
import os


class TestTorqueCalculator(unittest.TestCase):

    def __init__(self, methodName = "runTest"):
        super().__init__(methodName)

        test_dir = os.path.dirname(__file__)
        urdf_path = os.path.join(test_dir, "test_xml.urdf")
        with open(urdf_path, "r") as f:
            self.xml_string = f.read()

        self.robot_handler = TorqueCalculator(robot_description=self.xml_string)
        

    def test_init_torque_calculator(self):
        self.assertIsNotNone(self.robot_handler.model, "Model should be initialized")
        self.assertIsNotNone(self.robot_handler.geom_model,"Geom model should be initialized")
        self.assertIsNotNone(self.robot_handler.geom_data, "Model data should be initialized")
        print("✅ Initialization assertions passed")
        
    def test_get_root_name(self):
        root_name = self.robot_handler.get_root_joint_name(self.xml_string)
        self.assertEqual(self.robot_handler.get_root_name(), "base_footprint")
        self.assertEqual(root_name, self.robot_handler.get_root_name())
        print("✅ Root name assertion passed")

    def test_subtree(self):
        subtrees = self.robot_handler.get_subtrees()

        self.assertEqual(len(subtrees), 7)
        self.assertEqual(subtrees[0]["tip_link_name"], "wheel_front_left_link")
        self.assertEqual(subtrees[1]["tip_link_name"], "wheel_front_right_link")
        self.assertEqual(subtrees[2]["tip_link_name"], "wheel_rear_left_link")
        self.assertEqual(subtrees[3]["tip_link_name"], "wheel_rear_right_link")
        self.assertEqual(subtrees[4]["tip_link_name"], "gripper_left_outer_finger_left_link")
        self.assertEqual(subtrees[5]["tip_link_name"], "gripper_right_outer_finger_left_link")
        self.assertEqual(subtrees[6]["tip_link_name"], "head_2_link")
        print("✅ Subtrees assertion passed")

    def test_compute_mimic_joints(self):
        mimic_joints = self.robot_handler.mimic_joint_names
        self.assertEqual(len(mimic_joints), 10)
        self.assertIn("gripper_left_inner_finger_left_joint", mimic_joints)
        self.assertIn("gripper_right_inner_finger_left_joint", mimic_joints)
        self.assertNotIn("arm_1_joint", mimic_joints)
        print("✅ Mimic joints assertion passed")
    
    def 


if __name__ == "__main__":
    unittest.main()