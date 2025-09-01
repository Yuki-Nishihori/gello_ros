import time
from typing import Any, Dict, List
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from gello_ros.robots.robot import Robot


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    """Convert ROS Image message to numpy array without CvBridge for zero-copy efficiency.
    
    Args:
        msg: ROS Image message
        
    Returns:
        numpy array with shape (height, width, channels)
    """
    if msg is None:
        return None
        
    # Convert image data to numpy array
    dtype_map = {
        'rgb8': (np.uint8, 3),
        'bgr8': (np.uint8, 3), 
        'rgba8': (np.uint8, 4),
        'bgra8': (np.uint8, 4),
        'mono8': (np.uint8, 1),
        'mono16': (np.uint16, 1)
    }
    
    if msg.encoding not in dtype_map:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")
    
    dtype, channels = dtype_map[msg.encoding]
    
    # Convert bytes to numpy array
    image_array = np.frombuffer(msg.data, dtype=dtype)
    
    if channels == 1:
        # Grayscale image
        image_array = image_array.reshape((msg.height, msg.width))
    else:
        # Multi-channel image
        image_array = image_array.reshape((msg.height, msg.width, channels))
        
        # Convert BGR to RGB if necessary
        if msg.encoding == 'bgr8':
            image_array = image_array[:, :, ::-1]  # BGR -> RGB
        elif msg.encoding == 'bgra8':
            image_array = image_array[:, :, [2, 1, 0, 3]]  # BGRA -> RGBA
    
    return image_array



class Rate:
    def __init__(self, rate: float):
        self.last = time.time()
        self.rate = rate

    def sleep(self) -> None:
        while self.last + (1.0 / self.rate) > time.time():
            time.sleep(0.00001)
        self.last = time.time()


class RobotEnv(Node):
    def __init__(
        self,
        robot: Robot,
        control_rate_hz: float = 100.0,
        camera_names: List[str] = [],
        control_mode: str = "cartesian",
        node_name: str = "robot_env",
    ) -> None:
        super().__init__(node_name)
        assert control_mode in ["joint", "cartesian"]
        self._robot = robot
        self._rate = Rate(control_rate_hz)
        self._control_mode = control_mode
        
        # Camera handling - Zero-copy mode
        self.camera_names = camera_names
        self.camera_images = {}  # Store Image messages directly for zero-copy
        if self.camera_names:
            self.start_camera_subscribers()

    def start_camera_subscribers(self):
        """Start camera subscribers for all camera names"""
        for camera_name in self.camera_names:
            self.create_subscription(
                Image,
                f"/{camera_name}/color/image_raw",
                partial(self._camera_color_callback, camera_name),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Subscribed to /{camera_name}/color/image_raw in RobotEnv")

    def _camera_color_callback(self, camera_name, msg: Image):
        """Callback for the color image - Zero-copy mode: store Image message directly."""
        # Store the Image message directly for zero-copy access
        # Image-to-numpy conversion will be done on-demand in get_obs()
        self.camera_images[camera_name] = msg

    def robot(self) -> Robot:
        """Get the robot object.

        Returns:
            robot: the robot object.
        """
        return self._robot

    def __len__(self):
        return 0

    def step(self, action: Dict[str, np.ndarray]) -> Dict[str, Any]:
        
        if self._control_mode == "cartesian":
            # Ensure keys exist before accessing
            if "ee_pos" in action and "ee_quat" in action:
                pos_quat = np.concatenate([action["ee_pos"], action["ee_quat"]])
                self._robot.command_pose(pos_quat)
        elif self._control_mode == "joint":
            if "joint_positions" in action:
                self._robot.command_joint_state(action["joint_positions"])
        else:
            raise ValueError(f"Invalid control mode: {self._control_mode}")
        
        self._rate.sleep()
        return self.get_obs()
    
    def get_obs(self) -> Dict[str, Any]:
        """Get observation from the environment."""
        # Only spin robot node to update robot states
        # Camera updates are handled by the main spin loop in component mode
        rclpy.spin_once(self._robot, timeout_sec=0.001)
        rclpy.spin_once(self, timeout_sec=0.0001) # Process camera callbacks

        observations = {}
        
        # Add camera images to observations - Zero-copy conversion
        for name in self.camera_names:
            image_msg = self.camera_images.get(name)
            if image_msg is not None:
                observations[f"{name}_rgb"] = image_msg_to_numpy(image_msg)
            else:
                observations[f"{name}_rgb"] = None

        robot_obs = self._robot.get_observations()
        # It's safer to merge dictionaries
        observations.update(robot_obs)
        
        # Ensure essential keys are present
        assert "joint_positions" in observations
        assert "joint_velocities" in observations
        assert "ee_pos" in observations
        
        return observations


def main() -> None:
    pass


if __name__ == "__main__":
    main()