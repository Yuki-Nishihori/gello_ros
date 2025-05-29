#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    joint_offsets: Sequence[float]
    joint_signs: Sequence[int]
    gripper_config: Optional[Tuple[int, int, int]] = None

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 2000000,
        start_joints: Optional[np.ndarray] = None,
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            joint_signs=list(self.joint_signs),
            real=True,
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
            baudrate=baudrate,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(np.pi, 3*np.pi/2, np.pi, 3*np.pi/2, np.pi/2, 0),
        joint_signs=(1, 1, -1, 1, 1, 1),
    )
}


class GelloPublisher(Node, Agent):
    def __init__(self):
        super().__init__('gello_publisher_node')

        # Declare and get parameters
        port = self.declare_parameter("gello_port", "").get_parameter_value().string_value
        baudrate = self.declare_parameter("dynamixel_baudrate", 2000000).get_parameter_value().integer_value
        use_gripper = self.declare_parameter("use_gripper", False).get_parameter_value().bool_value
        control_hz = self.declare_parameter("control_hz", 10).get_parameter_value().integer_value
        self._mode = self.declare_parameter("mode", "unilateral_position").get_parameter_value().string_value

        assert os.path.exists(port), f"{port} does not exist"
        assert port in PORT_CONFIG_MAP, f"{port} not found in config map"

        config = PORT_CONFIG_MAP[port]
        if not use_gripper:
            config.gripper_config = None

        self._robot = config.make_robot(port=port, baudrate=baudrate)

        # Torque control parameters
        self.stall_torque = self.declare_parameter("stall_torque", 0.52).get_parameter_value().double_value
        self.stall_current = self.declare_parameter("stall_current", 1.5).get_parameter_value().double_value
        self.torque_constant = self.stall_torque / self.stall_current
        self.torque_rate = list(
            self.declare_parameter("torque_rate", [0.005] * 6).get_parameter_value().double_array_value
        )
        self.current_goal_constant = self.declare_parameter("current_goal_constant", 0.001).get_parameter_value().double_value

        # Robot control mode
        self._robot.set_control_mode("CURRENT_MODE")
        self._robot.set_torque_mode(True)
        self._robot.set_read_only(self._mode != "bilateral")

        # Publisher
        self.publisher = self.create_publisher(JointState, "/gello_joint_states", 10)

        # Timer
        self.create_timer(1.0 / control_hz, self.publish_joint_states)

    def publish_joint_states(self):
        joint_states = self._robot.get_joint_state()
        if joint_states is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint_{i+1}" for i in range(len(joint_states))]
        msg.position = joint_states.tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
