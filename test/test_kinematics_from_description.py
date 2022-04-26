#!/usr/bin/env python3

import os
import unittest
import numpy as np

from kinematics_from_description.kfd import KinematicsFromDescriptionTool as KFD

PKG = "kinematics_from_description"

test_robot_urdf = """<?xml version="1.0" ?>
<robot name="test_robot">
  <link name="test_robot/base_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="-0.0332053000 0.0008915770 0.0211913000"/>
      <mass value="0.395887"/>
      <inertia ixx="0.0010650000" ixy="-0.0000130300" ixz="0.0000018614" iyy="0.0003332000" iyz="0.0000409200" izz="0.0012080000"/>
    </inertial>
  </link>
  <joint name="waist" type="revolute">
    <axis xyz="0 0 1"/>
    <origin rpy="0 0 0" xyz="0 0 0.05085"/>
    <parent link="test_robot/base_link"/>
    <child link="test_robot/shoulder_link"/>
    <dynamics friction="0.1"/>
  </joint>
  <link name="test_robot/shoulder_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0000111169 -0.0003605640 0.0284598000"/>
      <mass value="0.072587"/>
      <inertia ixx="0.0000231000" ixy="0.0000000003" ixz="0.0000001606" iyy="0.0000253500" iyz="-0.0000000206" izz="0.0000144200"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="0 0 0.04225"/>
    <parent link="test_robot/shoulder_link"/>
    <child link="test_robot/upper_arm_link"/>
  </joint>
  <link name="test_robot/upper_arm_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0161976963 -0.0002929352 0.0877230000"/>
      <mass value="0.082923"/>
      <inertia ixx="0.0000800600" ixy="-0.0000002144" ixz="0.0000002982" iyy="0.0000745900" iyz="0.0000165700" izz="0.0000368500"/>
    </inertial>
  </link>
  <joint name="elbow" type="revolute">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="0.035 0 0.1"/>
    <parent link="test_robot/upper_arm_link"/>
    <child link="test_robot/forearm_link"/>
  </joint>
  <link name="test_robot/forearm_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0773720000 -0.0003324882 0.0000000000"/>
      <mass value="0.073058"/>
      <inertia ixx="0.0000533800" ixy="-0.0000003073" ixz="0.0000000000" iyy="0.0000165300" iyz="0.0000000000" izz="0.0000603500"/>
    </inertial>
  </link>
  <joint name="wrist_angle" type="revolute">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="0.1 0 0"/>
    <parent link="test_robot/forearm_link"/>
    <child link="test_robot/gripper_link"/>
  </joint>
  <link name="test_robot/gripper_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0446910000 0.0000000000 0.0113540000"/>
      <mass value="0.069929"/>
      <inertia ixx="0.0000226800" ixy="0.0000000000" ixz="0.0000000000" iyy="0.0000204400" iyz="0.0000008485" izz="0.0000197400"/>
    </inertial>
  </link>
  <joint name="ee_arm" type="fixed">
    <axis xyz="1 0 0"/>
    <origin rpy="0 0 0" xyz="0.063 0 0"/>
    <parent link="test_robot/gripper_link"/>
    <child link="test_robot/ee_arm_link"/>
  </joint>
  <link name="test_robot/ee_arm_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="gripper" type="continuous">
    <axis xyz="1 0 0"/>
    <origin rpy="0 0 0" xyz="0.0055 0 0"/>
    <parent link="test_robot/ee_arm_link"/>
    <child link="test_robot/gripper_prop_link"/>
  </joint>
  <link name="test_robot/gripper_prop_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0008460000 -0.0000019330 0.0000420000"/>
      <mass value="0.00434"/>
      <inertia ixx="0.0000005923" ixy="0.0000000000" ixz="0.0000003195" iyy="0.0000011156" iyz="-0.0000000004" izz="0.0000005743"/>
    </inertial>
  </link>
  <joint name="gripper_bar" type="fixed">
    <axis xyz="1 0 0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="test_robot/ee_arm_link"/>
    <child link="test_robot/gripper_bar_link"/>
  </joint>
  <link name="test_robot/gripper_bar_link">
    <inertial>
      <origin rpy="0 0 1.5707963267948966" xyz="0.0096870000 0.0000005660 0.0049620000"/>
      <mass value="0.034199"/>
      <inertia ixx="0.0000074125" ixy="-0.0000000008" ixz="-0.0000000006" iyy="0.0000284300" iyz="-0.0000013889" izz="0.0000286000"/>
    </inertial>
  </link>
  <joint name="ee_bar" type="fixed">
    <axis xyz="1 0 0"/>
    <origin rpy="0 0 0" xyz="0.023 0 0"/>
    <parent link="test_robot/gripper_bar_link"/>
    <child link="test_robot/fingers_link"/>
  </joint>
  <link name="test_robot/fingers_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="left_finger" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="test_robot/fingers_link"/>
    <child link="test_robot/left_finger_link"/>
  </joint>
  <link name="test_robot/left_finger_link">
    <inertial>
      <origin rpy="3.141592653589793 3.141592653589793 1.5707963267948966" xyz="0.0138160000 0.0000000000 0.0000000000"/>
      <mass value="0.016246"/>
      <inertia ixx="0.0000047310" ixy="-0.0000004560" ixz="0.0000000000" iyy="0.0000015506" iyz="0.0000000000" izz="0.0000037467"/>
    </inertial>
  </link>
  <joint name="right_finger" type="prismatic">
    <axis xyz="0 1 0"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="test_robot/fingers_link"/>
    <child link="test_robot/right_finger_link"/>
    <mimic joint="left_finger" multiplier="-1" offset="0"/>
  </joint>
  <link name="test_robot/right_finger_link">
    <inertial>
      <origin rpy="0 3.141592653589793 1.5707963267948966" xyz="0.0138160000 0.0000000000  0.0000000000"/>
      <mass value="0.016246"/>
      <inertia ixx="0.0000047310" ixy="0.0000004560" ixz="0.0000000000" iyy="0.0000015506" iyz="0.0000000000" izz="0.0000037467"/>
    </inertial>
  </link>
  <joint name="ee_gripper" type="fixed">
    <axis xyz="1 0 0"/>
    <origin rpy="0 0 0" xyz="0.027575 0 0"/>
    <parent link="test_robot/fingers_link"/>
    <child link="test_robot/ee_gripper_link"/>
  </joint>
  <link name="test_robot/ee_gripper_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>"""

Slist = np.array(
    [
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.0931, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.1931, 0.0, 0.035],
        [0.0, 1.0, 0.0, -0.1931, 0.0, 0.135],
    ]
).T

M = np.array(
    [
        [1.0, 0.0, 0.0, 0.248575],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.1931],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


class TestKFD(unittest.TestCase):
    def test_find_description_xml(self):
        tool = KFD({"body_frame": "", "space_frame": "", "namespace": ""})
        tool.load_desc_from_xml_string(
            """<?xml version="1.0"?><robot name="test_robot"></robot>"""
        )
        self.assertEqual(tool.robot.name, "test_robot")

    def test_find_description_file(self):
        filepath = os.path.dirname(os.path.abspath(__file__))
        tool = KFD({"body_frame": "", "space_frame": "", "namespace": ""})
        tool.load_desc_from_file(os.path.join(filepath, "test_robot.urdf"))
        self.assertEqual(tool.robot.name, "test_robot")

    def test_calc_M(self):
        tool = KFD(
            {
                "body_frame": "ee_gripper_link",
                "space_frame": "base_link",
                "namespace": "test_robot",
            }
        )
        tool.load_desc_from_xml_string(test_robot_urdf)
        tool.run()
        self.assertTrue(np.allclose(tool.get_M(), M))

    def test_calc_Slist(self):
        tool = KFD(
            {
                "body_frame": "ee_gripper_link",
                "space_frame": "base_link",
                "namespace": "test_robot",
            }
        )
        tool.load_desc_from_xml_string(test_robot_urdf)
        tool.run()
        self.assertTrue(np.allclose(tool.get_Slist(transposed=True), Slist))
        self.assertTrue(np.allclose(tool.get_Slist(), Slist.T))


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_kfd", TestKFD)
