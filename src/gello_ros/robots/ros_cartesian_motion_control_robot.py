import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_commander import MoveGroupCommander
import numpy as np
import tf_transformations as tf

from gello_ros.robots.robot import Robot
from ur_pykdl import ur_kinematics
from typing import Dict


class CartesianMotionControlRobot(Node, Robot):
    def __init__(self, use_gripper: bool = False):
        rclpy.init()
        Node.__init__(self, "ros2_cartesian_motion_robot")
        Robot.__init__(self)

        if use_gripper:
            self.get_logger().error("This implementation assumes no gripper.")
            exit()

        self.declare_parameters('', [
            ("joint_names_order", None),
            ("joint_max_vel", None),
            ("joint_pos_limits_upper", None),
            ("joint_pos_limits_lower", None),
            ("joint_trajectory_controller_command_topic", None),
            ("cartesian_motion_controller_command_topic", None),
            ("joint_states_topic", "/joint_states"),
            ("feedback_wrench_topic", "/wrench"),
            ("move_group_name", "manipulator"),
            ("ee_link", "tool0"),
        ])

        self.joint_names_order = self.get_parameter("joint_names_order").value
        self.joint_max_vel = self.get_parameter("joint_max_vel").value
        self.joint_pos_limits_upper = self.get_parameter("joint_pos_limits_upper").value
        self.joint_pos_limits_lower = self.get_parameter("joint_pos_limits_lower").value
        self.ee_link = self.get_parameter("ee_link").value

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("joint_trajectory_controller_command_topic").value,
            10
        )
        self.cartesian_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("cartesian_motion_controller_command_topic").value,
            10
        )

        self.ros_joint_state = None
        self._wrench = WrenchStamped()

        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_states_callback,
            10
        )
        self.create_subscription(
            WrenchStamped,
            self.get_parameter("feedback_wrench_topic").value,
            self.wrench_callback,
            10
        )

        self.move_group = MoveGroupCommander(self.get_parameter("move_group_name").value)
        self.kinematics = ur_kinematics()

        self._use_gripper = use_gripper
        self._speed_scale = 1
        self._min_traj_dur = 5.0 / 100

        self.get_logger().info("CartesianMotionControlRobot node initialized")

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

    def num_dofs(self) -> int:
        return 7 if self._use_gripper else 6

    def get_joint_state(self) -> np.ndarray:
        joint_positions_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.position))
        self.robot_joints = np.array([joint_positions_dict[name] for name in self.joint_names_order])
        return self.robot_joints

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        pose = self.kinematics.forward(joint_state, tip_link=self.ee_link)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]
        self.cartesian_pub.publish(msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = self.kinematics.forward(joints, tip_link=self.ee_link)
        gripper_pos = np.array([joints[-1]]) if self._use_gripper else np.zeros(1)
        wrench = np.array([
            self._wrench.wrench.force.x,
            self._wrench.wrench.force.y,
            self._wrench.wrench.force.z,
            self._wrench.wrench.torque.x,
            self._wrench.wrench.torque.y,
            self._wrench.wrench.torque.z,
        ])
        jacobian = self.move_group.get_jacobian_matrix(joints.tolist())
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos": pos_quat[:3],
            "ee_quat": pos_quat[3:],
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    node = CartesianMotionControlRobot(use_gripper=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
