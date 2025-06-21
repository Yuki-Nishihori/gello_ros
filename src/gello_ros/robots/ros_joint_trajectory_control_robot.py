#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Dict, Tuple

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import RobotTrajectory 

import tf.transformations

# --- 必要なライブラリをインポート ---
from trac_ik_python.trac_ik import IK as TRACK_IK_SOLVER
#FK計算のためにインポート
from ur_pykdl import ur_kinematics



class JointTrajectoryControlRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, use_gripper: bool = True, use_FT_sensor: bool = True):
        # グリッパー関連の初期化
        if use_gripper:
            print("caution: supposed only cobotta gripper")
            from gello_ros.robots.cobotta_gripper import CobottaGripper
            self.gripper = CobottaGripper()
            self.gripper.connect()
            print("gripper connected")
        
        # MoveIt Commanderの初期化はFK以外で利用するため維持
        move_group_name = rospy.get_param("~move_group_name", "manipulator")
        self.move_group = MoveGroupCommander(move_group_name)
        
        # --- 運動学ソルバーの初期化 ---
        rospy.loginfo("Initializing kinematics solvers...")
        base_link = self.move_group.get_planning_frame()
        self.ee_link = rospy.get_param("~ee_link")
        
        # 1. IKソルバーとしてTRAC-IKを初期化
        try:
            # ROSパラメータサーバからURDFを自動的に読み込む
            self.ik_solver = TRACK_IK_SOLVER(base_link, self.ee_link, timeout=0.05, solve_type="Distance")
            rospy.loginfo(f"TRAC-IK solver initialized from '{base_link}' to '{self.ee_link}'.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize TRAC-IK solver: {e}")

        # 2. FKソルバーとしてPyKDLを初期化
        try:
           
           self.kinematics = ur_kinematics(ee_link=self.ee_link)
        except Exception as e:
            rospy.logerr(f"Failed to initialize PyKDL FK solver: {e}")

        # ROSパラメータ取得、Publisher/Subscriber初期化
        self.joint_names_order = self.move_group.get_active_joints()
        self.joint_max_vel = rospy.get_param("~joint_max_vel")
        self.joint_pos_limits_upper = rospy.get_param("~joint_pos_limits_upper")
        self.joint_pos_limits_lower = rospy.get_param("~joint_pos_limits_lower")
        self.trajectory_publisher = rospy.Publisher(
            rospy.get_param("~joint_trajectory_controller_command_topic"),
            JointTrajectory,
            queue_size=1,
        )
        
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic"),
            JointState,
            self.joint_states_callback,
        )
        if use_FT_sensor:
            rospy.Subscriber(
                rospy.get_param("~feedback_wrench_topic"),
                WrenchStamped,
                self.wrench_callback,
            )

        self.control_hz = rospy.get_param("~control_hz", 100)
        self._min_traj_dur = 5.0 / self.control_hz
        self._speed_scale = 1
        self._use_gripper = use_gripper
        self._use_FTsensor = use_FT_sensor
        self.previous_joint_positions = None
        self.robot_joint_positions = None
        self.robot_joint_velocities = None
        self.robot_joint_torques = None

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

    def num_dofs(self) -> int:
        if self._use_gripper:
            return 7
        return 6

    def get_joint_state(self) -> np.ndarray:
        """
        ロボットの現在の関節状態（位置、速度、トルク）を取得します。
        """
        joint_positions_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.position))
        self.robot_joint_positions = np.array([joint_positions_dict.get(name, 0.0) for name in self.joint_names_order])

        if self.ros_joint_state.effort:
            joint_torques_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.effort))
            self.robot_joint_torques = np.array([joint_torques_dict.get(name, 0.0) for name in self.joint_names_order])
        else:
            rospy.logwarn_throttle(10, "JointState message does not contain effort information. Returning zeros.")
            self.robot_joint_torques = np.zeros(len(self.joint_names_order))

        if self._use_gripper:
            self.robot_joint_positions = np.append(self.robot_joint_positions, self.gripper.get_current_position())
            self.robot_joint_torques = np.append(self.robot_joint_torques, 0.0)

        if self.previous_joint_positions is not None:
            self.robot_joint_velocities = (self.robot_joint_positions - self.previous_joint_positions) * self.control_hz
        else:
            self.robot_joint_velocities = np.zeros_like(self.robot_joint_positions)
        
        self.previous_joint_positions = self.robot_joint_positions

        return self.robot_joint_positions, self.robot_joint_velocities, self.robot_joint_torques

    def command_pose(self, pose: np.ndarray) -> None:
        """
        指定された姿勢にエンドエフェクタを動かすためにIKを解き、ロボットに指令を送ります。
        IKの計算にはTRAC-IKを使用します。
        
        Args:
            pose (np.ndarray): ターゲットの姿勢 [x, y, z, qx, qy, qz, qw]
        """
        current_joint_positions, _, _ = self.get_joint_state()
        arm_joint_seed = list(current_joint_positions[:len(self.joint_names_order)])

        x, y, z, qx, qy, qz, qw = pose
        command_joints_arm_list = self.ik_solver.get_ik(
            arm_joint_seed, x, y, z, qx, qy, qz, qw
        )

        if command_joints_arm_list:
            command_joints_arm = np.array(command_joints_arm_list)
            
            if self._use_gripper:
                current_gripper_state = current_joint_positions[-1]
                command_joints = np.append(command_joints_arm, current_gripper_state)
            else:
                command_joints = command_joints_arm

            self.command_joint_state(command_joints)
        else:
            rospy.logwarn("TRAC-IK failed to find a solution for the requested pose.")

    def command_joint_state(self, joint_state: np.ndarray) -> None:
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
            dur.append(max(abs(command_joint_positions[i] - current_joint_positions[i]) / self.joint_max_vel[i], self._min_traj_dur,))
        
        if self._use_gripper:
            dynamixel_gripper_close_rate = command_joint_positions[-1]
            gripper_min_pos = self.gripper.get_min_position()
            gripper_max_pos = self.gripper.get_max_position()
            gripper_pos = gripper_min_pos + (gripper_max_pos - gripper_min_pos) * (1 - dynamixel_gripper_close_rate)
            self.gripper.move(position=gripper_pos, speed=50, force=6)
            
        point.time_from_start = rospy.Duration(max(dur) / self._speed_scale)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        j_pos, j_vel, j_effort = self.get_joint_state()
        
        try:
            pos_quat = self.kinematics.forward(j_pos, tip_link=self.ee_link)
        except Exception as e:
            rospy.logwarn_throttle(5, f"Could not compute FK using PyKDL: {e}")
            pos_quat = np.zeros(7)

        gripper_pos = np.array([j_pos[-1]]) if self._use_gripper else np.array([])
        
        if self._use_FTsensor:
            wrench = np.array([
                self._wrench.wrench.force.x, self._wrench.wrench.force.y, self._wrench.wrench.force.z,
                self._wrench.wrench.torque.x, self._wrench.wrench.torque.y, self._wrench.wrench.torque.z,
            ])
            jacobian = self.move_group.get_jacobian_matrix(list(j_pos[0:len(self.joint_names_order)]))
        else:
            wrench = np.zeros(6)
            jacobian = None
            
        return {
            "joint_positions": j_pos,
            "joint_velocities": j_vel,
            "joint_torques": j_effort,
            "ee_pos": pos_quat[:3],
            "ee_quat": pos_quat[3:],
            "ee_rot_matrix": tf.transformations.quaternion_matrix(pos_quat[3:])[:3, :3],
            "ee_euler": tf.transformations.euler_from_quaternion(pos_quat[3:]),
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }