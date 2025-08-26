#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from gello_ros.robots.robot import Robot

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kdl_parser_py.kdl_helper import KDLHelper
from pytracik.trac_ik import TracIK
import time


class JointTrajectoryControlRobot(Robot, Node):
    """A class representing a UR robot with joint trajectory control."""

    def __init__(self, use_gripper: bool = True, use_FT_sensor: bool = True):
        # Initialize ROS2 node
        Node.__init__(self, 'joint_trajectory_control_robot')
        
        # Gripper initialization
        if use_gripper:
            self.get_logger().info("Caution: supposed only cobotta gripper")
            try:
                from gello_ros.robots.cobotta_gripper import CobottaGripper
                self.gripper = CobottaGripper()
                self.gripper.connect()
                self.get_logger().info("Gripper connected")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize gripper: {e}")
                raise

        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Get parameters
        self.get_parameters()

        # Initialize kinematics
        self.get_logger().info("Initializing kinematics solvers...")
        
        # Initialize KDL helper for FK and Jacobian
        try:
            self.kdl_helper = KDLHelper(
                self.get_logger(),
                urdf_path=None,  # Will use robot_description parameter
                urdf_string=None,
                base_link="base_link",
                ee_link=self.ee_link
            )
            self.get_logger().info("KDL kinematics initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize KDL kinematics: {e}")
            raise

        # Initialize TracIK solver for IK
        try:
            self.ik_solver = TracIK(
                base_link_name="base_link",
                tip_link_name=self.ee_link,
                urdf_string=None,  # Will use robot_description parameter
                timeout=0.05,
                epsilon=1e-5,
                solver_type="Distance"
            )
            self.get_logger().info(f"TRAC-IK solver initialized from 'base_link' to '{self.ee_link}'")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TRAC-IK solver: {e}")
            # Continue without IK solver, but functionality will be limited

        # Initialize publisher
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.joint_trajectory_controller_command_topic,
            1
        )

        # Initialize subscribers
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            1
        )

        if use_FT_sensor:
            self.wrench_subscription = self.create_subscription(
                WrenchStamped,
                self.feedback_wrench_topic,
                self.wrench_callback,
                1
            )

        # Initialize variables
        self.ros_joint_state = None
        self._wrench = None
        self._use_gripper = use_gripper
        self._use_FTsensor = use_FT_sensor
        self.previous_joint_positions = None
        self.robot_joint_positions = None
        self.robot_joint_velocities = None
        self.robot_joint_torques = None
        
        # Control parameters
        self._min_traj_dur = 5.0 / self.control_hz
        self._speed_scale = 1

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        start_time = time.time()
        while self.ros_joint_state is None and rclpy.ok():
            if time.time() - start_time > 5:
                self.get_logger().error("Timeout waiting for joint_states_topic. Exiting.")
                raise TimeoutError("Joint states timeout")
            rclpy.spin_once(self, timeout_sec=0.1)

    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter("joint_names_order", [])
        self.declare_parameter("joint_max_vel", [])
        self.declare_parameter("joint_pos_limits_upper", [])
        self.declare_parameter("joint_pos_limits_lower", [])
        self.declare_parameter("joint_trajectory_controller_command_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("feedback_wrench_topic", "/ft_sensor/wrench")
        self.declare_parameter("control_hz", 100)
        self.declare_parameter("ee_link", "tool0")

    def get_parameters(self):
        """Get ROS2 parameters"""
        self.joint_names_order = self.get_parameter("joint_names_order").get_parameter_value().string_array_value
        self.joint_max_vel = self.get_parameter("joint_max_vel").get_parameter_value().double_array_value
        self.joint_pos_limits_upper = self.get_parameter("joint_pos_limits_upper").get_parameter_value().double_array_value
        self.joint_pos_limits_lower = self.get_parameter("joint_pos_limits_lower").get_parameter_value().double_array_value
        self.joint_trajectory_controller_command_topic = self.get_parameter("joint_trajectory_controller_command_topic").get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.feedback_wrench_topic = self.get_parameter("feedback_wrench_topic").get_parameter_value().string_value
        self.control_hz = self.get_parameter("control_hz").get_parameter_value().integer_value
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value

    def joint_states_callback(self, msg: JointState):
        """Joint states callback"""
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        """Wrench feedback callback"""
        self._wrench = msg

    def num_dofs(self) -> int:
        """Get the number of DOF of the robot"""
        if self._use_gripper:
            return 7
        return 6

    def get_joint_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get the current joint state (position, velocity, torque) of the robot.
        
        Returns:
            Tuple of (positions, velocities, efforts)
        """
        if self.ros_joint_state is None:
            return np.zeros(6), np.zeros(6), np.zeros(6)

        joint_positions_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.position))
        self.robot_joint_positions = np.array([joint_positions_dict.get(name, 0.0) for name in self.joint_names_order])

        if self.ros_joint_state.effort:
            joint_torques_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.effort))
            self.robot_joint_torques = np.array([joint_torques_dict.get(name, 0.0) for name in self.joint_names_order])
        else:
            if hasattr(self, '_effort_warning_logged') and not self._effort_warning_logged:
                self.get_logger().warning("JointState message does not contain effort information. Returning zeros.")
                self._effort_warning_logged = True
            self.robot_joint_torques = np.zeros(len(self.joint_names_order))

        if self._use_gripper and hasattr(self, 'gripper'):
            try:
                gripper_pos = self.gripper.get_current_position()
                self.robot_joint_positions = np.append(self.robot_joint_positions, gripper_pos)
                self.robot_joint_torques = np.append(self.robot_joint_torques, 0.0)
            except Exception as e:
                self.get_logger().error(f"Failed to get gripper position: {e}")

        if self.previous_joint_positions is not None:
            self.robot_joint_velocities = (self.robot_joint_positions - self.previous_joint_positions) * self.control_hz
        else:
            self.robot_joint_velocities = np.zeros_like(self.robot_joint_positions)
        
        self.previous_joint_positions = self.robot_joint_positions.copy()

        return self.robot_joint_positions, self.robot_joint_velocities, self.robot_joint_torques

    def command_pose(self, pose: np.ndarray) -> None:
        """
        Move the end-effector to the specified pose by solving IK and sending commands to the robot.
        Uses TracIK for IK computation.
        
        Args:
            pose (np.ndarray): Target pose [x, y, z, qx, qy, qz, qw]
        """
        if not hasattr(self, 'ik_solver'):
            self.get_logger().error("IK solver not available")
            return

        current_joint_positions, _, _ = self.get_joint_state()
        arm_joint_seed = current_joint_positions[:len(self.joint_names_order)].tolist()

        x, y, z, qx, qy, qz, qw = pose
        
        try:
            # Convert to rotation matrix for TracIK
            rotation = R.from_quat([qx, qy, qz, qw])
            rotation_matrix = rotation.as_matrix()
            
            command_joints_arm_list = self.ik_solver.ik(
                [x, y, z], rotation_matrix, seed_jnt_values=arm_joint_seed
            )

            if command_joints_arm_list is not None:
                command_joints_arm = np.array(command_joints_arm_list)
                
                if self._use_gripper:
                    current_gripper_state = current_joint_positions[-1]
                    command_joints = np.append(command_joints_arm, current_gripper_state)
                else:
                    command_joints = command_joints_arm

                self.command_joint_state(command_joints)
            else:
                self.get_logger().warning("TRAC-IK failed to find a solution for the requested pose.")
        except Exception as e:
            self.get_logger().error(f"Failed to solve IK: {e}")

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command joint trajectory"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names_order
        point = JointTrajectoryPoint()
        dur = []
        command_joint_positions = joint_state
        current_joint_positions, _, _ = self.get_joint_state()
        
        for i, name in enumerate(trajectory_msg.joint_names):
            pos = command_joint_positions[i]
            pos_lower = self.joint_pos_limits_lower[i]
            pos_upper = self.joint_pos_limits_upper[i]
            if pos < pos_lower:
                pos = pos_lower
            elif pos > pos_upper:
                pos = pos_upper
            point.positions.append(pos)
            dur.append(max(abs(command_joint_positions[i] - current_joint_positions[i]) / self.joint_max_vel[i], self._min_traj_dur))
        
        if self._use_gripper and hasattr(self, 'gripper'):
            try:
                dynamixel_gripper_close_rate = command_joint_positions[-1]
                gripper_min_pos = self.gripper.get_min_position()
                gripper_max_pos = self.gripper.get_max_position()
                gripper_pos = gripper_min_pos + (gripper_max_pos - gripper_min_pos) * (1 - dynamixel_gripper_close_rate)
                self.gripper.move(position=gripper_pos, speed=50, force=6)
            except Exception as e:
                self.get_logger().error(f"Failed to move gripper: {e}")
            
        duration_seconds = max(dur) / self._speed_scale
        point.time_from_start = Duration(seconds=duration_seconds).to_msg()
        trajectory_msg.points.append(point)
        
        # Set header timestamp
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get robot observations including kinematics and sensor data"""
        j_pos, j_vel, j_effort = self.get_joint_state()
        
        try:
            # Use KDL helper for forward kinematics
            pos, quat = self.kdl_helper.fk(j_pos.tolist())
            pos_quat = np.concatenate([pos, quat])
        except Exception as e:
            self.get_logger().warning(f"Could not compute FK using KDL helper: {e}")
            pos_quat = np.zeros(7)

        # Get gripper position
        if self._use_gripper:
            gripper_pos = np.array([j_pos[-1]]) if len(j_pos) > 0 else np.array([0.0])
        else:
            gripper_pos = np.array([0.0])

        # Get wrench data
        if self._wrench is not None:
            wrench = np.array([
                self._wrench.wrench.force.x,
                self._wrench.wrench.force.y,
                self._wrench.wrench.force.z,
                self._wrench.wrench.torque.x,
                self._wrench.wrench.torque.y,
                self._wrench.wrench.torque.z,
            ])
        else:
            wrench = np.zeros(6)

        # Get Jacobian using KDL helper
        try:
            jacobian = self.kdl_helper.jacobian(j_pos.tolist())
        except Exception as e:
            self.get_logger().warning(f"Could not compute Jacobian using KDL helper: {e}")
            jacobian = np.zeros((6, 6))

        # Convert quaternion to rotation matrix and euler angles
        if len(pos_quat) >= 7:
            try:
                rotation = R.from_quat(pos_quat[3:7])  # x,y,z,w format
                rot_matrix = rotation.as_matrix()
                euler_angles = rotation.as_euler('xyz')
            except Exception as e:
                self.get_logger().error(f"Failed to convert quaternion: {e}")
                rot_matrix = np.eye(3)
                euler_angles = np.zeros(3)
        else:
            rot_matrix = np.eye(3)
            euler_angles = np.zeros(3)

        return {
            "joint_positions": j_pos,
            "joint_velocities": j_vel,
            "joint_torques": j_effort,
            "gripper_position": gripper_pos,
            "ee_pos": pos_quat[:3] if len(pos_quat) >= 3 else np.zeros(3),
            "ee_quat": pos_quat[3:7] if len(pos_quat) >= 7 else np.array([0, 0, 0, 1]),
            "ee_rot_matrix": rot_matrix,
            "ee_euler": euler_angles,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rclpy.init()
    
    try:
        ros_robot = JointTrajectoryControlRobot(use_gripper=True, use_FT_sensor=True)
        rclpy.spin(ros_robot)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()