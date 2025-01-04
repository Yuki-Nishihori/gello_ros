from typing import Dict

import numpy as np

from gello_ros.robots.robot import Robot

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander


class JointTrajectoryControlRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, use_gripper: bool = True, use_FT_sensor: bool = True):
        if use_gripper:
            print("caution: supposed only cobotta gripper")
            if use_gripper:
                from gello_ros.robots.cobotta_gripper import CobottaGripper

                self.gripper = CobottaGripper()
                self.gripper.connect()
                print("gripper connected")

        self.joint_names_order = rospy.get_param("~joint_names_order")
        self.joint_max_vel = rospy.get_param("~joint_max_vel")
        self.joint_pos_limits_upper = rospy.get_param("~joint_pos_limits_upper")
        self.joint_pos_limits_lower = rospy.get_param("~joint_pos_limits_lower")
        self.trajectory_publisher = rospy.Publisher(
            rospy.get_param("~joint_trajectory_controller_command_topic"),
            JointTrajectory,
            queue_size=1,
        )
        self.move_group = MoveGroupCommander(
            rospy.get_param("~move_group_name", "manipulator")
        )
        rospy.Subscriber(
            rospy.get_param("~joint_states_topic"),
            JointState,
            self.joint_states_callback,
        )
        if use_FT_sensor:
            rospy.Subscriber(
                rospy.get_param("~wrench_topic"),
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
        self.robot_joint_positions = np.array(
            [joint_positions_dict[name] for name in self.joint_names_order]
        )
        if self._use_gripper:
            self.robot_joint_positions = np.append(
                self.robot_joint_positions, self.gripper.get_current_position()
            )

        # Calculate the joint velocities
        if self.previous_joint_positions is not None:
            self.robot_joint_velocities = (
                self.robot_joint_positions - self.previous_joint_positions
            ) * self.control_hz
        else:
            self.robot_joint_velocities = np.zeros_like(self.robot_joint_positions)
        self.previous_joint_positions = self.robot_joint_positions

        return self.robot_joint_positions, self.robot_joint_velocities

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names_order
        point = JointTrajectoryPoint()
        dur = []
        command_joint_positions = joint_state
        current_joint_positions, _ = self.get_joint_state()
        for i, name in enumerate(trajectory_msg.joint_names):
            pos = command_joint_positions[i]
            pos_lower = self.joint_pos_limits_lower[i]
            pos_upper = self.joint_pos_limits_upper[i]
            if pos < pos_lower:
                pos = pos_lower
            elif pos > pos_upper:
                pos = pos_upper
            point.positions.append(pos)

            dur.append(
                max(
                    abs(command_joint_positions[i] - current_joint_positions[i])
                    / self.joint_max_vel[i],
                    self._min_traj_dur,
                )
            )
        if self._use_gripper:
            dynamixel_gripper_close_rate = command_joint_positions[-1]
            gripper_min_pos = self.gripper.get_min_position()
            gripper_max_pos = self.gripper.get_max_position()
            gripper_pos = gripper_min_pos + (gripper_max_pos - gripper_min_pos) * (
                1 - dynamixel_gripper_close_rate
            )
            self.gripper.move(position=gripper_pos, speed=50, force=6)

        # set the target convergence time of the JTC to match the joint that tasks the longest time to move
        point.time_from_start = rospy.Duration(max(dur) / self._speed_scale)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)

    def get_observations(self) -> Dict[str, np.ndarray]:
        j_pos, j_vel = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([j_pos[-1]])
        if self._use_FTsensor:
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
            jacobian = self.move_group.get_jacobian_matrix(list(j_pos[0:5]))

        else:
            wrench = np.zeros(6)
            jacobian = None
        return {
            "joint_positions": j_pos,
            "joint_velocities": j_vel,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
            "ee_wrench": wrench,
            "jacobian": jacobian,
        }


def main():
    rospy.init_node("ros_robot")
    ros_robot = ROSRobot(use_gripper=False)


if __name__ == "__main__":
    main()
