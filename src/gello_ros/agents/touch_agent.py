import time
import numpy as np
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from omni_msgs.msg import OmniButtonEvent, OmniFeedback
import tf_transformations
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException


class TouchAgent(Node):
    def __init__(self):
        super().__init__('touch_agent')

        self.declare_parameters(
            namespace='',
            parameters=[
                ("touch_ee_pose_topic", "/touch/tip_pose"),
                ("touch_button_topic", "/touch/button"),
                ("touch_force_feedback_topic", "force_feedback"),
                ("touch_max_force", 1.5),
                ("force_scale_to_touch", 1.0),
                ("teleoperation_mode", "unilateral")
            ]
        )

        self.z_down_quat = tf_transformations.quaternion_from_euler(0, np.pi, 0)
        self._touch_current_pose = None
        self._touch_start_pose = None
        self._robot_start_pose = None
        self._robot_current_pose = None
        self._white_button = 0
        self._prev_white_button = 0
        self._grey_button = 0

        self.touch_max_force = self.get_parameter("touch_max_force").value
        self.force_scale_to_touch = self.get_parameter("force_scale_to_touch").value
        self.teleop_mode = self.get_parameter("teleoperation_mode").value

        # Publishers
        self.force_feedback_vis_pub = self.create_publisher(WrenchStamped, "force_feedback_vis", 10)
        self.force_feedback_pub = self.create_publisher(OmniFeedback, self.get_parameter("touch_force_feedback_topic").value, 10)

        # Subscribers
        self.create_subscription(PoseStamped, self.get_parameter("touch_ee_pose_topic").value, self.pose_callback, 10)
        self.create_subscription(OmniButtonEvent, self.get_parameter("touch_button_topic").value, self.button_callback, 10)

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for touch pose
        start_time = time.time()
        while self._touch_current_pose is None:
            if time.time() - start_time > 5:
                self.get_logger().error("Timeout waiting for touch pose topic.")
                rclpy.shutdown()
                return
            time.sleep(0.1)

        self.get_logger().info(f"Using {self.teleop_mode} teleoperation mode.")

    def pose_callback(self, msg: PoseStamped):
        pose_array = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self._touch_current_pose = pose_array

    def button_callback(self, msg: OmniButtonEvent):
        self._white_button = msg.white_button
        self._grey_button = msg.grey_button

    def transform_wrench(self, wrench_array, transform):
        wrench_array[:3] *= self.force_scale_to_touch
        wrench_array[3:] *= self.force_scale_to_touch
        wrench_array[:3] = np.clip(wrench_array[:3], -self.touch_max_force, self.touch_max_force)
        wrench_array[3:] = np.clip(wrench_array[3:], -self.touch_max_force, self.touch_max_force)

        rotation = tf_transformations.quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])[:3, :3]

        force_in_tool = wrench_array[:3]
        torque_in_tool = wrench_array[3:]

        force_in_base = rotation @ force_in_tool
        offset = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        torque_in_base = rotation @ torque_in_tool + np.cross(offset, force_in_base)

        vis = WrenchStamped()
        vis.header.frame_id = "touch_force"
        vis.wrench.force.x, vis.wrench.force.y, vis.wrench.force.z = force_in_base
        vis.wrench.torque.x, vis.wrench.torque.y, vis.wrench.torque.z = torque_in_base

        feedback = OmniFeedback()
        feedback.force.x, feedback.force.y, feedback.force.z = force_in_base

        return vis, feedback

    def calculate_pose_difference(self, start_pose, current_pose, robot_start_pose):
        pos_diff = current_pose[:3] - start_pose[:3]
        start_rot = tf_transformations.quaternion_matrix(start_pose[3:])[:3, :3]
        current_rot = tf_transformations.quaternion_matrix(current_pose[3:])[:3, :3]
        relative_rot = current_rot @ np.linalg.inv(start_rot)
        robot_start_rot = tf_transformations.quaternion_matrix(robot_start_pose[3:])[:3, :3]
        new_rot = relative_rot @ robot_start_rot
        new_quat = tf_transformations.quaternion_from_matrix(np.vstack((np.hstack((new_rot, [[0], [0], [0]])), [0, 0, 0, 1])))
        return np.concatenate((robot_start_pose[:3] + pos_diff, new_quat))

    def act(self, obs: Dict[str, np.ndarray], force_pose_update: bool = False) -> Dict[str, np.ndarray]:
        action_dict = {}
        pos_quat = np.concatenate([obs["ee_pos"], obs["ee_quat"]])
        action_pos_quat = np.zeros(7)

        try:
            transform = self.tf_buffer.lookup_transform("base", "tool0", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            wrench_vis, wrench_msg = self.transform_wrench(obs["ee_wrench"], transform)
            self.force_feedback_vis_pub.publish(wrench_vis)
            if self.teleop_mode == "bilateral":
                self.force_feedback_pub.publish(wrench_msg)
        except (LookupException, TimeoutException) as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")

        if self._prev_white_button == 0 and self._white_button == 1:
            self._robot_start_pose = pos_quat
            self._touch_start_pose = self._touch_current_pose
        elif self._prev_white_button == 1 and self._white_button == 0:
            self._robot_current_pose = pos_quat
        self._prev_white_button = self._white_button

        if self._white_button == 1 and self._grey_button == 1:
            vertical_pose = self.calculate_pose_difference(self._touch_start_pose, self._touch_current_pose, self._robot_start_pose)
            vertical_pose[3:] = self.z_down_quat
            action_pos_quat = vertical_pose
        elif self._white_button == 1:
            action_pos_quat = self.calculate_pose_difference(self._touch_start_pose, self._touch_current_pose, self._robot_start_pose)
        else:
            if self._robot_current_pose is None or force_pose_update:
                self._robot_current_pose = pos_quat
            action_pos_quat = self._robot_current_pose

        action_dict["joint_positions"] = np.zeros(6)
        action_dict["ee_pos"] = action_pos_quat[:3]
        action_dict["ee_quat"] = action_pos_quat[3:]
        action_dict["ee_rot_matrix"] = tf_transformations.quaternion_matrix(action_pos_quat[3:])[:3, :3]
        action_dict["ee_euler"] = tf_transformations.euler_from_quaternion(action_pos_quat[3:])
        return action_dict
