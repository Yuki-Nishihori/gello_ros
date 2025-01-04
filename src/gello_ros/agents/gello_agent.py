import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

from sensor_msgs.msg import JointState
import rospy
import moveit_commander



class GelloAgent(Agent):
    def __init__(
        self,
        topic_name: str = "/gello_joint_states",
    ):
        self
        gello_joint_states_sub = rospy.Subscriber(
            topic_name, JointState, self.joint_states_callback
        )
    def joint_states_callback(self, msg):
        self._joint_position = np.array(msg.position)
        
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
        return self._joint_position
