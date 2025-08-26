import os
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage


class RealSenseROS(Node):
    """
    A ROS2 node to subscribe to RealSense camera topics and provide the latest images.
    """

    def __init__(
        self,
        node_name: str = "realsense_subscriber",
        prefix: Optional[str] = None,
        flip: bool = False,
    ):
        """
        Initializes the node, parameters, and subscribers.

        Args:
            node_name (str): The name of the ROS2 node.
            prefix (str, optional): A prefix for the camera topics. Defaults to None.
            flip (bool, optional): Whether to flip the images vertically. Defaults to False.
        """
        super().__init__(node_name)

        # --- Configuration and State ---
        self._flip = flip
        self.bridge = CvBridge()
        self._color_image: Optional[np.ndarray] = None
        self._depth_image: Optional[np.ndarray] = None

        # --- Parameters ---
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_width", 640)
        height = self.get_parameter("camera_height").get_parameter_value().integer_value
        width = self.get_parameter("camera_width").get_parameter_value().integer_value

        self._empty_color_image = np.zeros((height, width, 3), dtype=np.uint8)
        self._empty_depth_image = np.zeros((height, width), dtype=np.uint16)

        # --- Topic Naming ---
        topic_prefix = "" if prefix is None else f"{prefix}/"

        # --- QoS Profile for Sensor Data ---
        # Use a reliable, best-effort QoS for camera streams to get the latest frame
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- ROS Subscribers ---
        self.color_subscriber = self.create_subscription(
            Image,
            f"{topic_prefix}camera/color/image_raw",
            self._color_callback,
            qos_profile=sensor_qos_profile,
        )

        # The depth subscriber remains commented out as in the original script.
        # self.depth_subscriber = self.create_subscription(
        #     CompressedImage,
        #     f"{topic_prefix}camera/depth/image_rect_raw/compressed", # Common ROS2 topic name
        #     self._depth_callback,
        #     qos_profile=sensor_qos_profile,
        # )
        
        self.get_logger().info("✅ RealSense subscriber node initialized.")

    def _color_callback(self, msg: Image):
        """Callback for the color image."""
        try:
            self._color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self._flip:
                self._color_image = cv2.rotate(self._color_image, cv2.ROTATE_180)
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")

    def _depth_callback(self, msg: CompressedImage):
        """Callback for the compressed depth image."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if self._flip:
                depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)
            self._depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def read_color_image(self) -> np.ndarray:
        """
        Gets the latest color image.

        Returns:
            np.ndarray: The latest color image, or an empty black image if none received.
        """
        if self._color_image is None:
            self.get_logger().warn("No color image received yet.", throttle_duration_sec=5)
            return self._empty_color_image
        return self._color_image.copy()

    def read_depth_image(self) -> np.ndarray:
        """
        Gets the latest depth image.

        Returns:
            np.ndarray: The latest depth image, or an empty zero image if none received.
        """
        if self._depth_image is None:
            self.get_logger().warn("No depth image received yet.", throttle_duration_sec=5)
            return self._empty_depth_image
        return self._depth_image.copy()


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)

    # Instantiate the node
    realsense_node = RealSenseROS()

    try:
        # Spin the node to process callbacks
        rclpy.spin(realsense_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        realsense_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()