from typing import Dict

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped


from ur_pykdl import ur_kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CartesianComplianceControlRobot(Robot):
    """A class representing a UR robot."""

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
            rospy.get_param("~cartesian_compliance_controller_command_topic"),
            PoseStamped,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic"),
            JointState,
            self.joint_states_callback,
        )
        rospy.Subscriber(
            rospy.get_param("~wrench_topic"),
            WrenchStamped,
            self.wrench_callback,
        )
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

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """

        # Create a dictionary for easy lookup
        joint_positions_dict = dict(
            zip(self.ros_joint_state.name, self.ros_joint_state.position)
        )
        # Reorder the joints according to self.joint_names
        self.robot_joints = np.array(
            [joint_positions_dict[name] for name in self.joint_names_order]
        )

        return self.robot_joints

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

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
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
        jacobian = self.move_group.get_jacobian_matrix(list(joints))
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rospy.init_node("ros_robot")
    ros_robot = CartesianComplianceControlRobot(use_gripper=False)


if __name__ == "__main__":
    main()
