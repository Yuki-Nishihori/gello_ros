#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from gello_ros.agents.spacemouse_agent import SpacemouseAgent, SpacemouseConfig


class SpacemouseAgentNode(Node):
    def __init__(self):
        super().__init__('spacemouse_agent_node')

        # Declare and retrieve parameters
        self.declare_parameter("robot_type", "ur5")
        self.declare_parameter("device_path", None)
        self.declare_parameter("invert_button", False)
        self.declare_parameter("verbose", True)

        robot_type = self.get_parameter("robot_type").get_parameter_value().string_value
        device_path = self.get_parameter("device_path").get_parameter_value().string_value
        invert_button = self.get_parameter("invert_button").get_parameter_value().bool_value
        verbose = self.get_parameter("verbose").get_parameter_value().bool_value

        self.agent = SpacemouseAgent(
            robot_type=robot_type,
            config=SpacemouseConfig(),
            device_path=device_path if device_path else None,
            verbose=verbose,
            invert_button=invert_button
        )

        self.latest_joint_state = None

        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.command_pub = self.create_publisher(Float64MultiArray, "/joint_command", 10)

        self.create_timer(1.0 / 20.0, self.publish_command)  # 20 Hz

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = {"joint_positions": np.array(msg.position)}

    def publish_command(self):
        if self.latest_joint_state is None:
            return
        try:
            action = self.agent.act(self.latest_joint_state)
        except Exception as e:
            self.get_logger().warn(f"Act failed: {e}")
            return
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.command_pub.publish(msg)
        self.get_logger().debug(f"Published action: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SpacemouseAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
