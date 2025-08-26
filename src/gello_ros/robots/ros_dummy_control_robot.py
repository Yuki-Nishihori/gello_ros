#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Dict, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

from gello_ros.robots.robot import Robot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped


class DummyControlRobot(Robot, Node):
    """A dummy robot controller for testing and debugging purposes."""

    def __init__(self, use_gripper: bool = True, use_FT_sensor: bool = True):
        # Initialize ROS2 node
        Node.__init__(self, 'dummy_control_robot')
        
        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Get parameters
        self.get_parameters()

        # Initialize variables
        self._use_gripper = use_gripper
        self._use_FTsensor = use_FT_sensor
        
        # Initialize joint states (6 DOF arm + gripper)
        self.num_joints = 6
        if self._use_gripper:
            self.num_joints += 1
            
        self.robot_joint_positions = np.zeros(self.num_joints)
        self.robot_joint_velocities = np.zeros(self.num_joints)
        self.robot_joint_torques = np.zeros(self.num_joints)
        
        # Initialize dummy EE pose (at origin with identity rotation)
        self.ee_position = np.array([0.5, 0.0, 0.3])  # Reasonable starting position
        self.ee_quaternion = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
        
        # Initialize dummy wrench
        self.wrench = np.zeros(6)
        
        # Control parameters
        self.control_hz = self.get_parameter("control_hz").get_parameter_value().integer_value
        self.previous_time = time.time()
        
        self.get_logger().info("Dummy control robot initialized")

    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter("joint_names_order", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self.declare_parameter("control_hz", 100)
        self.declare_parameter("ee_link", "tool0")

    def get_parameters(self):
        """Get ROS2 parameters"""
        self.joint_names_order = self.get_parameter("joint_names_order").get_parameter_value().string_array_value
        self.control_hz = self.get_parameter("control_hz").get_parameter_value().integer_value
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value

    def num_dofs(self) -> int:
        """Get the number of DOF of the robot"""
        return self.num_joints

    def get_joint_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get the current joint state (position, velocity, torque) of the robot.
        
        Returns:
            Tuple of (positions, velocities, efforts)
        """
        return self.robot_joint_positions.copy(), self.robot_joint_velocities.copy(), self.robot_joint_torques.copy()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command joint trajectory (dummy implementation that just stores the command)"""
        if len(joint_state) != self.num_joints:
            self.get_logger().warning(f"Expected {self.num_joints} joints, got {len(joint_state)}")
            return
            
        # Calculate velocities based on position change and time
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt > 0:
            self.robot_joint_velocities = (joint_state - self.robot_joint_positions) / dt
        
        # Update joint positions
        self.robot_joint_positions = joint_state.copy()
        
        # Update EE pose based on simple forward kinematics approximation
        self._update_ee_pose_from_joints()
        
        # Update time
        self.previous_time = current_time

    def command_pose(self, pose: np.ndarray) -> None:
        """
        Command end-effector pose (dummy implementation)
        
        Args:
            pose (np.ndarray): Target pose [x, y, z, qx, qy, qz, qw]
        """
        if len(pose) == 7:
            self.ee_position = pose[:3].copy()
            self.ee_quaternion = pose[3:].copy()
            
            # Simple inverse kinematics approximation
            self._update_joints_from_ee_pose()
        else:
            self.get_logger().warning(f"Expected pose of length 7, got {len(pose)}")

    def _update_ee_pose_from_joints(self):
        """Update EE pose based on joint positions (simplified forward kinematics)"""
        # Simple approximation: use joint positions to estimate EE position
        # This is a very simplified model for demonstration purposes
        
        # Use first 3 joints to approximate position
        if len(self.robot_joint_positions) >= 3:
            x = 0.3 + 0.2 * np.cos(self.robot_joint_positions[0])
            y = 0.2 * np.sin(self.robot_joint_positions[0])
            z = 0.2 + 0.3 * self.robot_joint_positions[1]
            self.ee_position = np.array([x, y, z])
        
        # Use last 3 joints for orientation (very simplified)
        if len(self.robot_joint_positions) >= 6:
            roll = self.robot_joint_positions[3]
            pitch = self.robot_joint_positions[4]
            yaw = self.robot_joint_positions[5]
            rotation = R.from_euler('xyz', [roll, pitch, yaw])
            self.ee_quaternion = rotation.as_quat()

    def _update_joints_from_ee_pose(self):
        """Update joint positions based on EE pose (simplified inverse kinematics)"""
        # Simple approximation: derive joint positions from EE pose
        # This is a very simplified model for demonstration purposes
        
        x, y, z = self.ee_position
        
        # Simple IK approximation
        if len(self.robot_joint_positions) >= 3:
            self.robot_joint_positions[0] = np.arctan2(y, x - 0.3)
            self.robot_joint_positions[1] = (z - 0.2) / 0.3
            self.robot_joint_positions[2] = 0.0  # Keep simple
        
        # Convert quaternion to euler for orientation
        if len(self.robot_joint_positions) >= 6:
            rotation = R.from_quat(self.ee_quaternion)
            euler = rotation.as_euler('xyz')
            self.robot_joint_positions[3:6] = euler

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get robot observations including kinematics and sensor data"""
        j_pos, j_vel, j_effort = self.get_joint_state()
        
        # Get EE pose
        pos_quat = np.concatenate([self.ee_position, self.ee_quaternion])
        
        # Get gripper position
        if self._use_gripper:
            gripper_pos = np.array([j_pos[-1]]) if len(j_pos) > 0 else np.array([0.0])
        else:
            gripper_pos = np.array([0.0])

        # Get wrench data (dummy values)
        wrench = self.wrench.copy()

        # Create dummy Jacobian (6x6 identity matrix)
        jacobian = np.eye(6)

        # Convert quaternion to rotation matrix and euler angles
        try:
            rotation = R.from_quat(self.ee_quaternion)
            rot_matrix = rotation.as_matrix()
            euler_angles = rotation.as_euler('xyz')
        except Exception as e:
            self.get_logger().error(f"Failed to convert quaternion: {e}")
            rot_matrix = np.eye(3)
            euler_angles = np.zeros(3)

        return {
            "joint_positions": j_pos,
            "joint_velocities": j_vel,
            "joint_torques": j_effort,
            "gripper_position": gripper_pos,
            "ee_pos": self.ee_position,
            "ee_quat": self.ee_quaternion,
            "ee_rot_matrix": rot_matrix,
            "ee_euler": euler_angles,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rclpy.init()
    
    try:
        dummy_robot = DummyControlRobot(use_gripper=True, use_FT_sensor=True)
        rclpy.spin(dummy_robot)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()