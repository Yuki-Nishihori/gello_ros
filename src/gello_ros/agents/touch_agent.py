import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

from geometry_msgs.msg import PoseStamped
import rospy
import moveit_commander



class TouchAgent(Agent):
    def __init__(
        self,
        topic_name: str = "/touch/pose",
    ):
        self._pose = None

        # Subscriber for pose topic
        self.pose_sub = rospy.Subscriber(
            topic_name, PoseStamped, self.pose_callback
        )

        # Wait for pose topic
        start_time = time.time()
        while self._pose is None:
            if time.time() - start_time > 5: # wait for 5 seconds
                rospy.logerr(f"Timeout waiting for {topic_name} topic. Exiting.")
                exit()
            rospy.sleep(0.1)

    def pose_callback(self, msg):
        pose_array = np.zeros(7)
        pose_array[0] = msg.pose.position.x + 0.1
        pose_array[1] = msg.pose.position.y + 0.1
        pose_array[2] = msg.pose.position.z + 0.1
        pose_array[3] = msg.pose.orientation.x
        pose_array[4] = msg.pose.orientation.y
        pose_array[5] = msg.pose.orientation.z
        pose_array[6] = msg.pose.orientation.w
        self._pose = pose_array

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
        return self._pose
