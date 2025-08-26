import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node



class GelloAgent(Agent, Node):
    def __init__(
        self,
        topic_name: str = "/gello_joint_states",
    ):
        # Initialize ROS2 node
        Node.__init__(self, 'gello_agent')
        
        # Initialize joint position variable
        self._joint_position = None
        # Create ROS2 subscriber
        self.gello_joint_states_sub = self.create_subscription(
            JointState, topic_name, self.joint_states_callback, 10
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
