import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from typing import Optional, Tuple
from camera_driver import DummyCamera, SavedCamera  # 上のコードのクラスを含むモジュール名に合わせてください


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('camera_type', 'dummy')
        self.declare_parameter('img_size', [480, 640])

        cam_type = self.get_parameter('camera_type').value
        img_size = tuple(self.get_parameter('img_size').value)

        if cam_type == 'dummy':
            self.camera = DummyCamera()
        elif cam_type == 'saved':
            self.camera = SavedCamera()
        else:
            raise ValueError(f"Unknown camera type: {cam_type}")

        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Image, '/camera/color', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth', 10)

        self.timer = self.create_timer(1.0 / 10.0, self.publish_images)  # 10 Hz

        self.get_logger().info(f"CameraPublisher started with {cam_type} camera.")

    def publish_images(self):
        color_img, depth_img = self.camera.read()
        color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='rgb8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='mono16')

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
