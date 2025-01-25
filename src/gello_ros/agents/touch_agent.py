import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent
import rospy
import moveit_commander
import tf.transformations



class TouchAgent(Agent):
    def __init__(
        self,
        pose_topic: str = "/touch/pose",
        button_topic: str = "/touch/button",
    ):
        self._touch_current_pose = None
        self._touch_start_pose = None
        self._robot_start_pose = None

        # Subscriber for pose topic
        self.pose_sub = rospy.Subscriber(
            pose_topic, PoseStamped, self.pose_callback
        )
        self.button_sub = rospy.Subscriber(
            button_topic, OmniButtonEvent, self.button_callback
        )
        self._button = 0
        self._prev_button = 0

        # Wait for pose topic
        start_time = time.time()
        while self._touch_current_pose is None:
            if time.time() - start_time > 5: # wait for 5 seconds
                rospy.logerr(f"Timeout waiting for {pose_topic} topic. Exiting.")
                exit()
            rospy.sleep(0.1)

    def pose_callback(self, msg):
        pose_array = np.zeros(7)
        pose_array[0] = msg.pose.position.x
        pose_array[1] = msg.pose.position.y
        pose_array[2] = msg.pose.position.z
        pose_array[3] = msg.pose.orientation.x
        pose_array[4] = msg.pose.orientation.y
        pose_array[5] = msg.pose.orientation.z
        pose_array[6] = msg.pose.orientation.w
        

        # # Convert quaternion to euler
        # euler = tf.transformations.euler_from_quaternion([
        #     msg.pose.orientation.x,
        #     msg.pose.orientation.y,
        #     msg.pose.orientation.z,
        #     msg.pose.orientation.w
        # ])

        # # Rotate roll (r) by 180 degrees
        # euler = (euler[0] + np.pi, euler[1], euler[2])

        # # Convert back to quaternion
        # quaternion = tf.transformations.quaternion_from_euler(*euler)

        # pose_array[3] = quaternion[0]
        # pose_array[4] = quaternion[1]
        # pose_array[5] = quaternion[2]
        # pose_array[6] = quaternion[3]

        self._touch_current_pose = pose_array
    
    def button_callback(self, msg):
        self._button = msg.white_button

            
        

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        # if self.mode == "bilateral":
        #     jacobian_inv = np.linalg.pinv(obs["jacobian"])
        #     wrench = obs["ee_wrench"]
        #     wrench[2] *= -1
        #     joint_torques = np.dot(jacobian_inv, wrench)
        #     joint_currents = joint_torques / self.torque_constant
        #     dynamixel_current_goals = joint_currents / self.current_goal_constant
        #     dynamixel_current_goals = np.round(
        #         dynamixel_current_goals * self.torque_rate
        #     ).astype(int)
        #     self._robot.command_joint_torque(dynamixel_current_goals)

        if self._prev_button == 0 and self._button == 1:
            self._robot_start_pose = obs["ee_pos_quat"]
            self._touch_start_pose = self._touch_current_pose
        self._prev_button = self._button
        if self._button == 1:   
            return (self._touch_current_pose - self._touch_start_pose)  + self._robot_start_pose
        else:
            return obs["ee_pos_quat"]