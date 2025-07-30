import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

class ImageOverlayNode(Node):
    def __init__(self):
        super().__init__('image_overlay_node')
        self.bridge = CvBridge()

        self.raw_image = None
        self.compressed_image = None

        self.raw_sub = self.create_subscription(Image, '/webcam/image_raw', self.raw_callback, 10)
        self.compressed_sub = self.create_subscription(CompressedImage, '/virtual_camera/image_raw/compressed', self.compressed_callback, 10)

        self.pub = self.create_publisher(Image, '/super_merge', 10)

    def raw_callback(self, msg):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.try_publish()
        except CvBridgeError as e:
            self.get_logger().error(f'Raw image conversion failed: {e}')

    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.compressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.try_publish()
        except Exception as e:
            self.get_logger().error(f'Compressed image decode failed: {e}')

    def try_publish(self):
        if self.raw_image is None or self.compressed_image is None:
            return

        # Resize overlay to match base image size
        overlay_resized = cv2.resize(self.compressed_image, (self.raw_image.shape[1], self.raw_image.shape[0]))

        # Overlay images (50% transparency)
        blended = cv2.addWeighted(self.raw_image, 0.7, overlay_resized, 0.3, 0)

        try:
            msg = self.bridge.cv2_to_imgmsg(blended, encoding='bgr8')
            self.pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert and publish: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()