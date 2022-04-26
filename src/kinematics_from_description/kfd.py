import scipy
import numpy as np
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R

"""
This module contains a class that can be used to calculate matrices required
for using the ModernRobotics Library. https://github.com/NxRLab/ModernRobotics
"""


class KinematicsFromDescriptionTool(object):
    def __init__(self, configs):
        self.body_frame_name = configs["body_frame"]
        self.space_frame_name = configs["space_frame"]
        self.robot_namespace = configs.get("namespace")

    def load_desc_from_param(self):
        """
        Gets the URDF from the robot_description parameter on the ROS parameter
        server
        """
        key = "%s/robot_description" % self.robot_namespace
        try:
            self.robot = URDF.from_parameter_server(key=key)
        except KeyError:
            print(
                (
                    "Error: `%s` not found on the ROS parameter server. "
                    "Check that it is loaded and under the right namespace."
                )
                % key
            )
            exit(1)

    def load_desc_from_file(self, file_path):
        """
        Gets the URDF from the robot's URDF file
        """
        self.robot = URDF.from_xml_file(file_path)

    def load_desc_from_xml_string(self, xml_string):
        """
        Gets the URDF from an xml string containing the URDF
        """
        self.robot = URDF.from_xml_string(xml_string)

    def _populate(self):
        """
        Populates relevant variables and lists
        """
        self.links = self.robot.links
        self.joints = self.robot.joints

        # init empty lists
        self.link_list = []
        self.tf_list = []
        self.axis_list = []
        self.joint_list = []
        self.rev_joint_list = []
        self.rev_origin_list = []

        # find ee link in urdf tree
        self.body_frame = self._search(
            self.links, "%s/%s" % (self.robot_namespace, self.body_frame_name), "name"
        )

        self.space_frame = self._search(
            self.links, "%s/%s" % (self.robot_namespace, self.space_frame_name), "name"
        )

        if self.body_frame is None or self.space_frame is None:
            raise ValueError("Body Frame or Space Frame are empty")

    def get_M(self):
        """
        Returns the homogeneous transform matrix

        @return M
        """
        return self.M

    def get_Slist(self, transposed=False):
        """
        Returns the list of joint screw axes in the world frame

        @return Slist
        """
        if transposed:
            return self.Slist
        else:
            return self.Slist.T

    def run(self):
        """
        Builds relevant lists and calculates the M and Slist matrices
        """
        self._populate()
        self._build_link_lists()
        self._calc_M()
        self._calc_Slist()

    def _calc_M(self):
        """
        Calculates the homogeneous matrix describing the pose of the end
            effector in SE(3)
        """
        M = np.eye(4, dtype="float64")
        for T in self.tf_list:
            M = np.dot(M, T)
        self.M = M

    def _calc_Slist(self):
        """
        Calculates the list of screw axes
        """
        num_axes = len(self.axis_list)  # number of revolute axes defines Slist shape
        Slist = np.zeros((6, num_axes))
        for i in range(num_axes):
            w = np.array(self.axis_list[i], dtype="float64")
            q = np.array(self.rev_origin_list[i], dtype="float64")
            v = np.cross(-w, q)
            Slist[:, i] = np.hstack((w, v))
        self.Slist = Slist

    def _build_link_lists(self):
        """
        Builds the transformation and link lists

        Iterates from the specified end effector link to the base frame
        """
        self.link_list.insert(0, self.body_frame)
        self.tf_list.insert(0, self._get_parent_transform(self.body_frame))
        current_link = self._get_parent_link(self.body_frame)

        # iterate backwards through link/joint tree up to base
        while current_link is not None and (current_link.name != self.space_frame.name):

            self.link_list.insert(0, current_link)

            parent_joint = self._get_parent_joint(current_link)
            self.joint_list.insert(0, parent_joint)

            current_tf_to_parent = self._get_parent_transform(current_link)
            self.tf_list.insert(0, current_tf_to_parent)

            # keep track of revolute joints and their axes
            if parent_joint.type == "revolute":
                self.rev_joint_list.insert(0, parent_joint)
                self.axis_list.insert(0, self._get_parent_axis(current_link))

            current_link = self._get_parent_link(current_link)

        joint_pose = np.eye(4, dtype="float64")
        for i in range(len(self.joint_list)):
            joint_pose = np.dot(joint_pose, self.tf_list[i])
            if self.joint_list[i].type == "revolute":
                self.rev_origin_list.append(joint_pose[:3, 3])

    def _get_joint_translation(self, joint):
        """
        Returns the translation of the given joint

        @param joint - joint to find the translation of
        @return translation from parent link to child link
        """
        if joint.origin is None or joint.origin.xyz is None:
            return np.array([0, 0, 0], dtype="float64")
        return np.array(joint.origin.xyz)

    def _get_joint_rotation(self, joint):
        """
        Returns the rotation of the given joint

        @param joint - joint to find the rotation of
        @return rotation from parent link to child link
        """
        if joint.origin is None or joint.origin.rpy is None:
            if scipy.__version__ >= "1.4.0":
                return R.from_euler("xyz", [0, 0, 0]).as_matrix()
            else:
                return R.from_euler("xyz", [0, 0, 0]).as_dcm()
        rpy = joint.origin.rpy
        if scipy.__version__ >= "1.4.0":
            return R.from_euler("xyz", rpy).as_matrix()
        else:
            return R.from_euler("xyz", rpy).as_dcm()

    def _get_parent_transform(self, link):
        """
        Returns the transformation from parent to child given the child link

        @param link - child link of joint
        @return transformation from parent to child
        """
        joint = self._get_parent_joint(link)
        T = np.eye(4, dtype="float64")
        if joint is None:
            return T
        else:
            T[:3, :3] = self._get_joint_rotation(joint)
            T[:3, 3] = self._get_joint_translation(joint)
            return T

    def _get_parent_link(self, link):
        """
        Returns the parent link given its child

        @param link - child link to find the parent of
        @return parent link to given link
        @note returns None if there is no parent link
        """
        parent_joint = self._get_parent_joint(link)
        if parent_joint is None:
            return None
        return self._search(self.links, parent_joint.parent, "name")

    def _get_parent_joint(self, link):
        """
        Returns the parent joint given its child link

        @param link - child link to find the parent joint of
        @return parent joint to given link
        """
        joint = self._search(self.joints, link.name, "child")
        return joint

    def _get_parent_axis(self, link):
        """
        Returns the parent joint axis given its child

        @param link - child link to the find the parent joint's axis of
        @return axis about which the parent joint rotates
        """
        if self._get_parent_joint(link).type != "revolute":
            return None
        else:
            axis = self._get_parent_joint(link).axis
            return axis

    def _search(self, list_, key, attr):
        """
        Helper function to perform a key search through a list of objects

        @param list_ - list of objects to search through
        @param key - term to search for
        @param attr - attribute to look for key in for each object
        @return the first matching object that has an attribute matching the key
        @note returns None if no object attribute matches the key
        """
        result = [element for element in list_ if getattr(element, attr) == key]
        if len(result) > 0:
            return result[0]
        else:
            return None
