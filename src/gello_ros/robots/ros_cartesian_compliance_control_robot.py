import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
import tf_transformations as tf

# Robot, ur_kinematics, MoveGroupCommander は別途インポート
from your_package.kinematics import ur_kinematics
from your_package.moveit_interface import MoveGroupCommander
from your_package.robot_base import Robot


class CartesianComplianceControlRobot(Node, Robot):
    def __init__(self):
        Node.__init__(self, "ros2_robot_node")
        Robot.__init__(self)

        self.declare_parameters(
            namespace="",
            parameters=[
                ("joint_names_order", None),
                ("joint_max_vel", None),
                ("joint_pos_limits_upper", None),
                ("joint_pos_limits_lower", None),
                ("joint_trajectory_controller_command_topic", None),
                ("cartesian_compliance_controller_command_topic", None),
                ("joint_states_topic", "/joint_states"),
                ("feedback_wrench_topic", "/wrench"),
                ("compliance_control_wrench_zero_service", None),
                ("feedback_wrench_zero_service", None),
                ("move_group_name", "manipulator"),
                ("ee_link", "tool0"),
            ],
        )

        # パラメータ取得
        self.joint_names_order = self.get_parameter("joint_names_order").value
        self.joint_max_vel = self.get_parameter("joint_max_vel").value
        self.joint_pos_limits_upper = self.get_parameter("joint_pos_limits_upper").value
        self.joint_pos_limits_lower = self.get_parameter("joint_pos_limits_lower").value
        self.ee_link = self.get_parameter("ee_link").value

        self.trajectory_pub = self.create_publisher(
            JointState, self.get_parameter("joint_trajectory_controller_command_topic").value, 10
        )
        self.cartesian_pub = self.create_publisher(
            PoseStamped, self.get_parameter("cartesian_compliance_controller_command_topic").value, 10
        )

        # サブスクライバ
        self.ros_joint_state = None
        self._wrench = None

        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_states_callback,
            10,
        )

        self.create_subscription(
            WrenchStamped,
            self.get_parameter("feedback_wrench_topic").value,
            self.wrench_callback,
            10,
        )

        # MoveIt初期化
        self.move_group = MoveGroupCommander(self.get_parameter("move_group_name").value)
        self.kinematics = ur_kinematics()

        # FTセンサのゼロリセット
        self.call_empty_service("compliance_control_wrench_zero_service")
        self.call_empty_service("feedback_wrench_zero_service")

        self.get_logger().info("CartesianComplianceControlRobot is initialized.")

    def call_empty_service(self, param_name):
        srv_name = self.get_parameter(param_name).value
        client = self.create_client(Empty, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for service {srv_name}")
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f"Service {srv_name} called successfully.")
        else:
            self.get_logger().error(f"Failed to call service: {srv_name}")

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

    def get_joint_state(self) -> np.ndarray:
        joint_positions_dict = dict(zip(self.ros_joint_state.name, self.ros_joint_state.position))
        return np.array([joint_positions_dict[name] for name in self.joint_names_order])

    def command_pose(self, pose: np.ndarray):
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

    def get_observations(self) -> dict:
        joints = self.get_joint_state()
        pos_quat = self.kinematics.forward(joints, tip_link=self.ee_link)
        gripper_pos = np.array([0.0])
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
            "joint_torques": joints,
            "gripper_position": gripper_pos,
            "ee_pos": pos_quat[:3],
            "ee_quat": pos_quat[3:],
            "ee_rot_matrix": tf.quaternion_matrix(pos_quat[3:])[:3, :3],
            "ee_euler": tf.euler_from_quaternion(pos_quat[3:]),
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main(args=None):
    rclpy.init(args=args)
    node = CartesianComplianceControlRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
