import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import Optional, Tuple


class RealSenseROS2(Node):
    def __init__(self, prefix: Optional[str] = None, flip: bool = False):
        super().__init__('realsense_ros2_camera')

        self._flip = flip
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_width', 640)

        height = self.get_parameter('camera_height').get_parameter_value().integer_value
        width = self.get_parameter('camera_width').get_parameter_value().integer_value

        self._empty_color_image = np.zeros((height, width, 3), dtype=np.uint8)
        self._color_image = None
        self.bridge = CvBridge()

        if prefix is None:
            prefix = ""
        else:
            prefix = prefix + "_"

        color_topic = f"/{prefix}camera/color/image_raw"

        self.create_subscription(
            Image,
            color_topic,
            self._color_callback,
            10
        )

        self.get_logger().info(f"Subscribed to {color_topic}")

    def _color_callback(self, msg: Image):
        try:
            self.get_logger().info("Received color image.")
            self._color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self._flip:
                self._color_image = cv2.rotate(self._color_image, cv2.ROTATE_180)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def read(self, img_size: Optional[Tuple[int, int]] = None) -> np.ndarray:
        """Return the latest color image."""
        if self._color_image is None:
            self.get_logger().warn("No color image received.")
            return self._empty_color_image
        if img_size is not None:
            return cv2.resize(self._color_image, img_size)
        return self._color_image


def main():
    rclpy.init()
    node = RealSenseROS2(prefix="my", flip=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            image = node.read()
            cv2.imshow("color", image)
            if cv2.waitKey(1) == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
