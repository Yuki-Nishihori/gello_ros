import numpy as np
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander

from gello_ros.robots.robot import Robot


class JointTrajectoryControlRobot(Node, Robot):
    def __init__(self, use_gripper: bool = True, use_FT_sensor: bool = True):
        super().__init__('joint_trajectory_control_robot')

        if use_gripper:
            from gello_ros.robots.cobotta_gripper import CobottaGripper
            self.gripper = CobottaGripper()
            self.gripper.connect()
            self.get_logger().info("Gripper connected")

        self.joint_names_order = self.declare_parameter("joint_names_order", []).value
        self.joint_max_vel = np.array(self.declare_parameter("joint_max_vel", []).value)
        self.joint_pos_limits_upper = np.array(self.declare_parameter("joint_pos_limits_upper", []).value)
        self.joint_pos_limits_lower = np.array(self.declare_parameter("joint_pos_limits_lower", []).value)

        traj_topic = self.declare_parameter("joint_trajectory_controller_command_topic").value
        wrench_topic = self.declare_parameter("feedback_feedback_wrench_topic", "/wrench/filtered").value
        joint_states_topic = self.declare_parameter("joint_states_topic", "/joint_states").value
        move_group_name = self.declare_parameter("move_group_name", "manipulator").value
        self.control_hz = self.declare_parameter("control_hz", 100).value

        self.trajectory_publisher = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.create_subscription(JointState, joint_states_topic, self.joint_states_callback, 10)
        if use_FT_sensor:
            self.create_subscription(WrenchStamped, wrench_topic, self.wrench_callback, 10)

        self.move_group = MoveGroupCommander(move_group_name)

        self._min_traj_dur = 5.0 / self.control_hz
        self._speed_scale = 1.0
        self._use_gripper = use_gripper
        self._use_FTsensor = use_FT_sensor
        self.previous_joint_positions = None
        self.robot_joint_positions = None
        self.robot_joint_velocities = None
        self.ros_joint_state = None
        self._wrench = WrenchStamped()

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

    def num_dofs(self) -> int:
        return 7 if self._use_gripper else 6

    def get_joint_state(self) -> np.ndarray:
        joint_positions_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.position))
        self.robot_joint_positions = np.array([joint_positions_dict[name] for name in self.joint_names_order])

        if self._use_gripper:
            self.robot_joint_positions = np.append(self.robot_joint_positions, self.gripper.get_current_position())

        if self.previous_joint_positions is not None:
            self.robot_joint_velocities = (self.robot_joint_positions - self.previous_joint_positions) * self.control_hz
        else:
            self.robot_joint_velocities = np.zeros_like(self.robot_joint_positions)

        self.previous_joint_positions = self.robot_joint_positions

        return self.robot_joint_positions, self.robot_joint_velocities

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names_order
        point = JointTrajectoryPoint()
        dur = []

        current_joint_positions, _ = self.get_joint_state()
        for i, name in enumerate(trajectory_msg.joint_names):
            pos = np.clip(joint_state[i], self.joint_pos_limits_lower[i], self.joint_pos_limits_upper[i])
            point.positions.append(pos)
            duration = max(abs(pos - current_joint_positions[i]) / self.joint_max_vel[i], self._min_traj_dur)
            dur.append(duration)

        if self._use_gripper:
            rate = joint_state[-1]
            min_pos, max_pos = self.gripper.get_min_position(), self.gripper.get_max_position()
            gripper_pos = min_pos + (max_pos - min_pos) * (1 - rate)
            self.gripper.move(position=gripper_pos, speed=50, force=6)

        point.time_from_start.sec = int(max(dur) / self._speed_scale)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        j_pos, j_vel = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([j_pos[-1]])

        if self._use_FTsensor:
            wrench = np.array([
                self._wrench.wrench.force.x,
                self._wrench.wrench.force.y,
                self._wrench.wrench.force.z,
                self._wrench.wrench.torque.x,
                self._wrench.wrench.torque.y,
                self._wrench.wrench.torque.z,
            ])
            jacobian = self.move_group.get_jacobian_matrix(list(j_pos[0:6]))
        else:
            wrench = np.zeros(6)
            jacobian = None

        return {
            "joint_positions": j_pos,
            "joint_velocities": j_vel,
            "ee_pos": pos_quat[:3],
            "ee_quat": pos_quat[3:],
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rclpy.init()
    node = JointTrajectoryControlRobot(use_gripper=False)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
