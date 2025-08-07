#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StaticImagePublisher(Node):
    def __init__(self):
        super().__init__('static_image_publisher')

        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory

        pkg_share = Path(get_package_share_directory('mr_pkg'))
        default_image = pkg_share / 'images' / 'frame2_01_17.jpg'  # pick any frame as default

        # Get parameter (relative or absolute)
        image_path_param = self.declare_parameter(
            'image_path',
            'images/frame2_01_17.jpg'  # default relative to pkg_share
        ).get_parameter_value().string_value

        image_path = Path(image_path_param)
        if not image_path.is_absolute():
            image_path = pkg_share / image_path

        # Load once
        frame = cv2.imread(str(image_path))
        if frame is None:
            self.get_logger().error(f'Cannot read image: {image_path}')
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher = self.create_publisher(Image, 'image_raw', 10)

        # 30 Hz timer
        self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info(f'Publishing {image_path} on /image_raw at 30 fps')

    def timer_callback(self):
        self.publisher.publish(self.msg)

def main():
    rclpy.init()
    node = StaticImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()