import numpy as np
from typing import Dict, Optional

from rclpy.node import Node
from sensor_msgs.msg import JointState

from gello_ros.agents.agent import Agent


class GelloAgent(Agent):
    def __init__(self, node: Node, topic_name: str = "/gello_joint_states"):
        """
        Args:
            node (Node): ROS 2 Node インスタンス
            topic_name (str): JointState トピック名
        """
        self.node = node
        self._joint_position: Optional[np.ndarray] = None

        self.subscriber = self.node.create_subscription(
            JointState,
            topic_name,
            self.joint_states_callback,
            10
        )

    def joint_states_callback(self, msg: JointState):
        self._joint_position = np.array(msg.position)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        """
        観測から行動を返す（ここでは関節位置をそのまま返す）

        Args:
            obs: 観測情報（未使用）

        Returns:
            np.ndarray: 現在の関節位置
        """
        if self._joint_position is None:
            self.node.get_logger().warn("Joint position not yet received.")
            return np.zeros(6)  # or raise exception
        return self._joint_position
