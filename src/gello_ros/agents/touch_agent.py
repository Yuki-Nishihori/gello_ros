import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

from geometry_msgs.msg import PoseStamped, WrenchStamped
from omni_msgs.msg import OmniButtonEvent, OmniFeedback
import rospy
import moveit_commander
import tf.transformations
import tf2_ros
import tf2_geometry_msgs


class TouchAgent(Agent):
    def __init__(
        self,
    ):
        ee_pose_topic = rospy.get_param("~touch_ee_pose_topic")
        button_topic = rospy.get_param("~touch_button_topic")
        force_feedback_topic = rospy.get_param("~touch_force_feedback_topic", "force_feedback")
        self.max_force = rospy.get_param("~max_force", 1.5)
        self.force_scale = rospy.get_param("~force_scale", 1.0)
        self._touch_current_pose = None
        self._touch_start_pose = None
        self._robot_start_pose = None
        self.z_down_quat = tf.transformations.quaternion_from_euler(0, np.pi, 0) 
        self.teleop_mode = rospy.get_param("~teleoperation_mode", "unilateral")
        if self.teleop_mode not in ["unilateral", "bilateral"]:
            rospy.logerr(f"Invalid communication mode: {self.mode}. Exiting.")
            exit()
        else:
            rospy.loginfo(f"Using {self.teleop_mode} teleoperation mode.")

        # Publisher for force feedback
        self.force_feedback_vis_pub = rospy.Publisher(
            "force_feedback_vis", WrenchStamped, queue_size=10
        )
        self.force_feedback_pub = rospy.Publisher(
            force_feedback_topic, OmniFeedback, queue_size=10
        )
        

        # Subscriber for pose topic
        self.pose_sub = rospy.Subscriber(
            ee_pose_topic, PoseStamped, self.pose_callback
        )
        self.button_sub = rospy.Subscriber(
            button_topic, OmniButtonEvent, self.button_callback
        )
        # self._force_transform_sub = rospy.Subscriber(
        #     "force_transform", WrenchStamped, self.force_transform_callback
        # )
        self._white_button = 0
        self._prev_white_button = 0
        self._grey_button = 0

        # Wait for pose topic
        start_time = time.time()
        while self._touch_current_pose is None:
            if time.time() - start_time > 5: # wait for 5 seconds
                rospy.logerr(f"Timeout waiting for {ee_pose_topic} topic. Exiting.")
                exit()
            rospy.sleep(0.1)

        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_callback(self, msg):
        pose_array = np.zeros(7)
        pose_array[0] = msg.pose.position.x
        pose_array[1] = msg.pose.position.y
        pose_array[2] = msg.pose.position.z
        pose_array[3] = msg.pose.orientation.x
        pose_array[4] = msg.pose.orientation.y
        pose_array[5] = msg.pose.orientation.z
        pose_array[6] = msg.pose.orientation.w
        self._touch_current_pose = pose_array
    
    def button_callback(self, msg):
        self._white_button = msg.white_button
        self._grey_button = msg.grey_button

    def transform_wrench(self, wrench_array, transform):
        # Apply scaling to the force and torque
        wrench_array[:3] *= self.force_scale
        wrench_array[3:] *= self.force_scale

        # Apply max force limit
        wrench_array[:3] = np.clip(wrench_array[:3], -self.max_force, self.max_force)
        wrench_array[3:] = np.clip(wrench_array[3:], -self.max_force, self.max_force)

        wrench_in_tool = WrenchStamped()
        wrench_in_tool.wrench.force.x = wrench_array[0]
        wrench_in_tool.wrench.force.y = wrench_array[1]
        wrench_in_tool.wrench.force.z = wrench_array[2]
        wrench_in_tool.wrench.torque.x = wrench_array[3]
        wrench_in_tool.wrench.torque.y = wrench_array[4]
        wrench_in_tool.wrench.torque.z = wrench_array[5]
        force_in_tool = np.array([wrench_in_tool.wrench.force.x,
                                  wrench_in_tool.wrench.force.y,
                                  wrench_in_tool.wrench.force.z])
        torque_in_tool = np.array([wrench_in_tool.wrench.torque.x,
                                   wrench_in_tool.wrench.torque.y,
                                   wrench_in_tool.wrench.torque.z])

        rotation = tf.transformations.quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])[:3, :3]

        force_in_base = np.dot(rotation, force_in_tool)
        tool_offset = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])
        torque_in_base = np.dot(rotation, torque_in_tool) + np.cross(tool_offset, force_in_base)

        wrench_in_base_vis = WrenchStamped()
        wrench_in_base_vis.header.frame_id = "touch_force"
        wrench_in_base_vis.wrench.force.x = force_in_base[0]
        wrench_in_base_vis.wrench.force.y = force_in_base[1]
        wrench_in_base_vis.wrench.force.z = force_in_base[2]
        wrench_in_base_vis.wrench.torque.x = torque_in_base[0]
        wrench_in_base_vis.wrench.torque.y = torque_in_base[1]
        wrench_in_base_vis.wrench.torque.z = torque_in_base[2]

        wrench_in_base = OmniFeedback()
        wrench_in_base.force.x = force_in_base[0]
        wrench_in_base.force.y = force_in_base[1]
        wrench_in_base.force.z = force_in_base[2]

        return wrench_in_base_vis, wrench_in_base

    def calculate_pose_difference(self, start_pose, current_pose, robot_start_pose):
        pos_diff = current_pose[:3] - start_pose[:3]
        start_quat = start_pose[3:]
        current_quat = current_pose[3:]

        # Convert quaternions to rotation matrices
        start_rot = tf.transformations.quaternion_matrix(start_quat)[:3, :3]
        current_rot = tf.transformations.quaternion_matrix(current_quat)[:3, :3]

        # Calculate the relative rotation
        relative_rot = np.dot(current_rot, np.linalg.inv(start_rot))

        # Convert the relative rotation back to a quaternion
        relative_quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((relative_rot, [[0], [0], [0]])), [0, 0, 0, 1])))

        # Apply the relative rotation to the robot start pose quaternion
        robot_start_quat = robot_start_pose[3:]
        robot_start_rot = tf.transformations.quaternion_matrix(robot_start_quat)[:3, :3]
        new_rot = np.dot(relative_rot, robot_start_rot)
        new_quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((new_rot, [[0], [0], [0]])), [0, 0, 0, 1])))

        return np.concatenate((robot_start_pose[:3] + pos_diff, new_quat))

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        try:
            transform = self.tf_buffer.lookup_transform("base", "tool0", rospy.Time(0), rospy.Duration(1.0))
            wrench_in_base_vis,wrench_in_base = self.transform_wrench(obs["ee_wrench"], transform)
            self.force_feedback_vis_pub.publish(wrench_in_base_vis)
        except tf2_ros.TransformException as ex:
            rospy.logwarn(f"TransformException: {ex}")
            return obs["ee_pos_quat"]
        if self.teleop_mode == "bilateral":
            self.force_feedback_pub.publish(wrench_in_base)
        if self._prev_white_button == 0 and self._white_button == 1:
            self._robot_start_pose = obs["ee_pos_quat"]
            self._touch_start_pose = self._touch_current_pose
        self._prev_white_button = self._white_button
        if self._white_button == 1 and self._grey_button == 1:
            vertical_pose = self.calculate_pose_difference(self._touch_start_pose, self._touch_current_pose, self._robot_start_pose)
            vertical_pose[3:] = self.z_down_quat
            return vertical_pose
        elif self._white_button == 1:
            return self.calculate_pose_difference(self._touch_start_pose, self._touch_current_pose, self._robot_start_pose)
        else:
            print(obs["ee_pos_quat"])
            return obs["ee_pos_quat"]