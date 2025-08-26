#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello_ros.agents.agent import Agent
from gello_ros.robots.dynamixel import DynamixelRobot
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

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
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
            baudrate=baudrate,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            2 * np.pi / 2,
            3 * np.pi / 2,
            2 * np.pi / 2,
            3 * np.pi / 2,
            1 * np.pi / 2,
            0 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None,  # (7, 113.091015625, 71.291015625),
    ),
    # Cobotta
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YXAT-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         3 * np.pi / 2,
    #         4 * np.pi / 2,
    #         2 * np.pi / 2,
    #         0 * np.pi / 2,
    #         3 * np.pi / 2 + np.pi / 4,
    #         3 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, -1, 1, -1, 1),
    #     gripper_config=(7, 96, 54),
    # ),
    # FR3
    # "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0": DynamixelRobotConfig(
    #     joint_ids=(1, 2, 3, 4, 5, 6),
    #     joint_offsets=(
    #         5 * np.pi / 2,
    #         3 * np.pi / 2,
    #         2 * np.pi / 2,
    #         3 * np.pi / 2,
    #         1 * np.pi / 2,
    #         0 * np.pi / 2,
    #         2 * np.pi / 2,
    #         6 * np.pi / 2,
    #         3 * np.pi / 2,
    #         3 * np.pi / 2,
    #     ),
    #     joint_signs=(1, 1, -1, 1, 1, 1),
    #     gripper_config=(7, 113.091015625, 71.291015625),
    # ),
}


class GelloPublisher(Agent, Node):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
        mode: Optional[str] = "unilateral_position",
    ):
        # Initialize ROS2 node
        Node.__init__(self, 'gello_publisher')
        # Declare ROS2 parameters
        self.declare_parameter("dynamixel_baudrate", 2000000)
        self.declare_parameter("use_gripper", False)
        self.declare_parameter("stall_torque", 0.52)
        self.declare_parameter("stall_current", 1.5)
        self.declare_parameter("torque_rate", [0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
        self.declare_parameter("current_goal_constant", 0.001)
        self.declare_parameter("control_hz", 10)
        self.declare_parameter("gello_port", "")
        
        # Get parameters
        baudrate = self.get_parameter("dynamixel_baudrate").get_parameter_value().integer_value
        use_gripper = self.get_parameter("use_gripper").get_parameter_value().bool_value
        
        if dynamixel_config is not None:
            if not use_gripper:
                dynamixel_config.gripper_config = None
            self._robot = dynamixel_config.make_robot(
                port=port, baudrate=baudrate, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"
            config = PORT_CONFIG_MAP[port]
            if not use_gripper:
                config.gripper_config = None
            self._robot = config.make_robot(
                port=port, baudrate=baudrate, start_joints=start_joints
            )
        self._mode = mode

        # Set constants for torque feedback
        self.stall_torque = self.get_parameter("stall_torque").get_parameter_value().double_value
        self.stall_current = self.get_parameter("stall_current").get_parameter_value().double_value
        self.torque_constant = self.stall_torque / self.stall_current
        self.torque_rate = self.get_parameter("torque_rate").get_parameter_value().double_array_value
        self.current_goal_constant = self.get_parameter("current_goal_constant").get_parameter_value().double_value

        # Set control mode
        self._robot.set_control_mode(
            "CURRENT_MODE"
        )  # POSITION_MODE,CURRENT_BASED_POSITION_MODE
        # Set torque
        self._robot.set_torque_mode(True)
        if self._mode == "bilateral":
            self._robot.set_read_only(False)
        else:
            self._robot.set_read_only(True)
        # Create ROS2 publisher and timer
        self.gello_pub = self.create_publisher(JointState, '/gello_joint_states', 10)
        self.publish_rate = self.get_parameter("control_hz").get_parameter_value().integer_value
        
        # Create timer for publishing joint states
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_states_callback)

    def publish_joint_states_callback(self):
        """Timer callback for publishing joint states"""
        joint_states = self._robot.get_joint_state()
        if joint_states is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'joint_{i+1}' for i in range(len(joint_states))]  # Optional: customize joint names
            msg.position = joint_states  # Assuming joint_states contains joint positions
            # If you have velocity and effort data:
            # msg.velocity = [0.0] * len(joint_states)
            # msg.effort = [0.0] * len(joint_states)
            self.gello_pub.publish(msg)
            
    def start_publishing(self):
        """Start the ROS2 node spinning"""
        rclpy.spin(self)
        
def main():
    rclpy.init()
    
    try:
        # Get gello_port parameter from command line or default
        import sys
        port = None
        if len(sys.argv) > 1:
            port = sys.argv[1]
        
        agent = GelloPublisher(port=port)
        agent.start_publishing()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()