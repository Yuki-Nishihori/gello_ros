from typing import Dict

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_srvs.srv import Empty

from ur_pykdl import ur_kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import tf.transformations

class CartesianImpedanceControlRobot(Robot):
    """A class representing a UR robot with Cartesian impedance control."""

    def __init__(
        self,
        use_gripper: bool = False,
    ):
        if use_gripper:
            print("supposed only no gripper")
            exit()

        self.joint_names_order = rospy.get_param("~joint_names_order")
        self.joint_max_vel = rospy.get_param("~joint_max_vel")
        self.joint_pos_limits_upper = rospy.get_param("~joint_pos_limits_upper")
        self.joint_pos_limits_lower = rospy.get_param("~joint_pos_limits_lower")
        self.trajectory_publisher = rospy.Publisher(
            rospy.get_param("~joint_trajectory_controller_command_topic"),
            JointTrajectory,
            queue_size=1,
        )
        self.cartesian_command_publisher = rospy.Publisher(
            rospy.get_param("~cartesian_impedance_controller_command_topic"),
            PoseStamped,
            queue_size=1,
        )

        # Wait for joint_states_topic
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic"),
            JointState,
            self.joint_states_callback,
        )
        self.ros_joint_state = None
        start_time = time.time()
        while self.ros_joint_state is None:
            if time.time() - start_time > 5: # wait for 5 seconds
                rospy.logerr(f"Timeout waiting for joint_states_topic. Exiting.")
                exit()
            rospy.sleep(0.1)

        # Wait for feedback_wrench_topic
        rospy.Subscriber(
            rospy.get_param("~feedback_wrench_topic"),
            WrenchStamped,
            self.wrench_callback,
        )
        self._wrench = None
        start_time = time.time()
        while self._wrench is None:
            if time.time() - start_time > 5:
                rospy.logerr(f"Timeout waiting for feedback_wrench_topic. Exiting.")
                exit()
            rospy.sleep(0.1)
        

        # Zero reset feedback FT sensor offset
        wrench_zero_service_name = rospy.get_param("~feedback_wrench_zero_service")
        rospy.wait_for_service(wrench_zero_service_name)
        try:
            self.cotroller_wrench_zero_service = rospy.ServiceProxy(wrench_zero_service_name, Empty)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            exit()
        rospy.loginfo("Zero reset feedback FT sensor offset")
        self.cotroller_wrench_zero_service.call()
        rospy.sleep(1)


        self.move_group = MoveGroupCommander(
            rospy.get_param("move_group_name", "manipulator")
        )
        self.kinematics = ur_kinematics()
        self.ee_link = rospy.get_param("~ee_link")

        control_freq = 100
        self._min_traj_dur = 5.0 / control_freq
        self._speed_scale = 1
        self._use_gripper = use_gripper

    def joint_states_callback(self, msg: JointState):
        self.ros_joint_state = msg

    def wrench_callback(self, msg: WrenchStamped):
        self._wrench = msg

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get the current state of the leader robot.

        Returns:
            tuple: (positions, velocities, efforts) of the robot joints.
        """
        # Create dictionaries for easy lookup
        joint_positions_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.position)
        )
        joint_velocities_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.velocity)
        )
        joint_efforts_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.effort)
        )
        
        # Reorder the joints according to self.joint_names
        positions = np.array(
            [joint_positions_dict[name] for name in self.joint_names_order]
        )
        velocities = np.array(
            [joint_velocities_dict[name] for name in self.joint_names_order]
        )
        efforts = np.array(
            [joint_efforts_dict[name] for name in self.joint_names_order]
        )
        
        self.robot_joints = positions

        return positions, velocities, efforts

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        pose = self.kinematics.forward(joint_state, tip_link=self.ee_link)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        pose_stamped.pose.orientation.x = pose[3]
        pose_stamped.pose.orientation.y = pose[4]
        pose_stamped.pose.orientation.z = pose[5]
        pose_stamped.pose.orientation.w = pose[6]

        self.cartesian_command_publisher.publish(pose_stamped)

    def command_pose(self, pose: np.ndarray) -> None:
        """Command the leader robot to a given pose.

        Args:
            pose (np.ndarray): The pose to command the leader robot to.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
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
        joint_positions, joint_velocities, joint_efforts = self.get_joint_state()
        pos_quat = self.kinematics.forward(joint_positions, tip_link=self.ee_link)
        gripper_pos = np.array([joint_positions[-1]])
        
        wrench = np.array(
            [
                self._wrench.wrench.force.x,
                self._wrench.wrench.force.y,
                self._wrench.wrench.force.z,
                self._wrench.wrench.torque.x,
                self._wrench.wrench.torque.y,
                self._wrench.wrench.torque.z,
            ]
        )
        jacobian = self.move_group.get_jacobian_matrix(list(joint_positions))
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_torques": joint_efforts,
            "gripper_position": gripper_pos,
            "ee_pos": pos_quat[:3],
            "ee_quat": pos_quat[3:],
            "ee_rot_matrix": tf.transformations.quaternion_matrix(pos_quat[3:])[:3, :3],
            "ee_euler": tf.transformations.euler_from_quaternion(pos_quat[3:]),
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rospy.init_node("ros_robot")
    ros_robot = CartesianImpedanceControlRobot(use_gripper=False)


if __name__ == "__main__":
    main()