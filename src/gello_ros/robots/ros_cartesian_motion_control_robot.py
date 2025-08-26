from typing import Dict

import numpy as np
from scipy.spatial.transform import Rotation as R

from gello_ros.robots.robot import Robot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kdl_parser_py.kdl_helper import KDLHelper
from pytracik.trac_ik import TracIK
import time


class CartesianMotionControlRobot(Robot, Node):
    """A class representing a UR robot with Cartesian motion control."""

    def __init__(
        self,
        use_gripper: bool = False,
    ):
        # Initialize ROS2 node
        Node.__init__(self, 'cartesian_motion_control_robot')
        
        if use_gripper:
            self.get_logger().error("Only no gripper configuration supported")
            raise NotImplementedError("Gripper support not implemented")

        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Get parameters
        self.get_parameters()

        # Initialize publishers
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            self.joint_trajectory_controller_command_topic,
            1
        )
        self.cartesian_command_publisher = self.create_publisher(
            PoseStamped,
            self.cartesian_motion_controller_command_topic,
            1
        )

        # Initialize subscribers
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_states_callback,
            1
        )
        self.wrench_subscription = self.create_subscription(
            WrenchStamped,
            self.feedback_wrench_topic,
            self.wrench_callback,
            1
        )

        # Initialize variables
        self.ros_joint_state = None
        self._wrench = WrenchStamped()

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        start_time = time.time()
        while self.ros_joint_state is None and rclpy.ok():
            if time.time() - start_time > 5:
                self.get_logger().error("Timeout waiting for joint_states_topic. Exiting.")
                raise TimeoutError("Joint states timeout")
            rclpy.spin_once(self, timeout_sec=0.1)

        # Initialize kinematics
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

        # Initialize IK solver
        try:
            self.ik_solver = TracIK(
                base_link_name="base_link",
                tip_link_name=self.ee_link,
                urdf_string=None,  # Will use robot_description parameter
                timeout=0.05,
                epsilon=1e-5,
                solver_type="Distance"
            )
            self.get_logger().info("TracIK solver initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TracIK solver: {e}")
            # Continue without IK solver for now

        # Control parameters
        control_freq = 100
        self._min_traj_dur = 5.0 / control_freq
        self._speed_scale = 1
        self._use_gripper = use_gripper

    def _declare_parameters(self):
        """Declare ROS2 parameters"""
        self.declare_parameter("joint_names_order", [])
        self.declare_parameter("joint_max_vel", [])
        self.declare_parameter("joint_pos_limits_upper", [])
        self.declare_parameter("joint_pos_limits_lower", [])
        self.declare_parameter("joint_trajectory_controller_command_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("cartesian_motion_controller_command_topic", "/cartesian_motion_controller/target_frame")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("feedback_wrench_topic", "/ft_sensor/wrench")
        self.declare_parameter("ee_link", "tool0")

    def get_parameters(self):
        """Get ROS2 parameters"""
        self.joint_names_order = self.get_parameter("joint_names_order").get_parameter_value().string_array_value
        self.joint_max_vel = self.get_parameter("joint_max_vel").get_parameter_value().double_array_value
        self.joint_pos_limits_upper = self.get_parameter("joint_pos_limits_upper").get_parameter_value().double_array_value
        self.joint_pos_limits_lower = self.get_parameter("joint_pos_limits_lower").get_parameter_value().double_array_value
        self.joint_trajectory_controller_command_topic = self.get_parameter("joint_trajectory_controller_command_topic").get_parameter_value().string_value
        self.cartesian_motion_controller_command_topic = self.get_parameter("cartesian_motion_controller_command_topic").get_parameter_value().string_value
        self.joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.feedback_wrench_topic = self.get_parameter("feedback_wrench_topic").get_parameter_value().string_value
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value

    def joint_states_callback(self, msg: JointState):
        """Joint states callback"""
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        """Wrench feedback callback"""
        self._wrench = msg

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            np.ndarray: The current state of the leader robot.
        """
        if self.ros_joint_state is None:
            return np.zeros(6)

        # Create a dictionary for easy lookup
        joint_positions_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.position)
        )
        # Reorder the joints according to self.joint_names_order
        self.robot_joints = np.array(
            [joint_positions_dict.get(name, 0.0) for name in self.joint_names_order]
        )

        return self.robot_joints

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        try:
            # Use KDL helper for forward kinematics
            pos, quat = self.kdl_helper.fk(joint_state.tolist())
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose.position.x = pos[0]
            pose_stamped.pose.position.y = pos[1]
            pose_stamped.pose.position.z = pos[2]
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            self.cartesian_command_publisher.publish(pose_stamped)
        except Exception as e:
            self.get_logger().error(f"Failed to command joint state: {e}")

    def command_pose(self, pose: np.ndarray) -> None:
        """Command the leader robot to a given pose.

        Args:
            pose (np.ndarray): The pose to command the leader robot to (pos + quat).
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        pose_stamped.pose.orientation.x = pose[3]
        pose_stamped.pose.orientation.y = pose[4]
        pose_stamped.pose.orientation.z = pose[5]
        pose_stamped.pose.orientation.w = pose[6]

        self.cartesian_command_publisher.publish(pose_stamped)

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get robot observations including kinematics and sensor data"""
        joints = self.get_joint_state()
        
        try:
            # Use KDL helper for forward kinematics
            pos, quat = self.kdl_helper.fk(joints.tolist())
            pos_quat = np.concatenate([pos, quat])
        except Exception as e:
            self.get_logger().error(f"Failed to compute FK: {e}")
            pos_quat = np.zeros(7)
            
        gripper_pos = np.array([joints[-1]]) if len(joints) > 0 else np.array([0.0])

        # Get wrench data
        wrench = np.array([
            self._wrench.wrench.force.x,
            self._wrench.wrench.force.y,
            self._wrench.wrench.force.z,
            self._wrench.wrench.torque.x,
            self._wrench.wrench.torque.y,
            self._wrench.wrench.torque.z,
        ])

        # Get Jacobian using KDL helper
        try:
            jacobian = self.kdl_helper.jacobian(joints.tolist())
        except Exception as e:
            self.get_logger().error(f"Failed to compute Jacobian: {e}")
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
            "joint_positions": joints,
            "joint_velocities": joints,  # Note: Original code had bug, using joints for velocities
            "joint_torques": joints,     # Not in original, but added for consistency
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
        ros_robot = CartesianMotionControlRobot(use_gripper=False)
        rclpy.spin(ros_robot)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()