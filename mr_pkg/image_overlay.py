import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

VIRTUAL_CAMERA_TOPIC = '/virtual_camera/image_raw/compressed'
VIRTUAL_DEPTH_TOPIC = '/virtual_camera/depth/compressed'  # For future Unity depth
PHYSICAL_CAMERA_TOPIC = '/webcam/image_raw'
PHYSICAL_DEPTH_TOPIC = '/webcam/depth'  # New depth topic from webcam publisher
MERGED_IMAGE_TOPIC = '/mixed_reality/camera'
MERGED_FRAME_RATE = 30  # Hz

class DepthAwareImageOverlayNode(Node):
    def __init__(self):
        super().__init__('depth_aware_image_overlay_node')
        self.bridge = CvBridge()
        
        # Image and depth storage
        self.physical_image = None
        self.virtual_image = None
        self.physical_depth = None
        self.virtual_depth = None
        
        # ROS subscriptions and publishers
        self.physical_image_sub = self.create_subscription(
            Image, PHYSICAL_CAMERA_TOPIC, self.physical_image_callback, 10)
        self.physical_depth_sub = self.create_subscription(
            Image, PHYSICAL_DEPTH_TOPIC, self.physical_depth_callback, 10)
        self.virtual_image_sub = self.create_subscription(
            CompressedImage, VIRTUAL_CAMERA_TOPIC, self.virtual_image_callback, 10)
        self.virtual_depth_sub = self.create_subscription(
            CompressedImage, VIRTUAL_DEPTH_TOPIC, self.virtual_depth_callback, 10)
        
        self.merged_pub = self.create_publisher(Image, MERGED_IMAGE_TOPIC, MERGED_FRAME_RATE)
        
        # Parameters
        self.declare_parameter('occlusion_tolerance', 0.05)
        self.declare_parameter('black_threshold', 30)
        self.declare_parameter('debug_mode', False)
        
        self.get_logger().info("Depth-aware overlay node initialized")

    def physical_image_callback(self, msg):
        """Callback for physical camera image"""
        try:
            self.physical_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.try_publish_merged()
        except CvBridgeError as e:
            self.get_logger().error(f'Physical image conversion failed: {e}')

    def physical_depth_callback(self, msg):
        """Callback for physical camera depth"""
        try:
            self.physical_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.try_publish_merged()
        except CvBridgeError as e:
            self.get_logger().error(f'Physical depth conversion failed: {e}')

    def virtual_image_callback(self, msg):
        """Callback for virtual camera image"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.virtual_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.try_publish_merged()
        except Exception as e:
            self.get_logger().error(f'Virtual image decode failed: {e}')

    def virtual_depth_callback(self, msg):
        """Callback for virtual depth data from Unity (future implementation)"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
            # Convert to float and normalize (adjust based on Unity's depth encoding)
            self.virtual_depth = depth_img.astype(np.float32) / 255.0
            self.try_publish_merged()
        except Exception as e:
            self.get_logger().error(f'Virtual depth decode failed: {e}')

    def create_occlusion_mask(self, virtual_img, virtual_depth, physical_depth):
        """Create mask for occluded virtual objects"""
        if physical_depth is None:
            # No physical depth available - render all virtual content
            self.get_logger().warn_once("No physical depth data available")
            return np.ones((virtual_img.shape[0], virtual_img.shape[1]), dtype=np.uint8) * 255
        
        if virtual_depth is None:
            # No virtual depth - assume all virtual objects are at medium distance
            self.get_logger().warn_once("No virtual depth data available, using fallback")
            # Create a simple fallback: assume virtual objects are at 0.5 depth
            virtual_depth = np.full((virtual_img.shape[0], virtual_img.shape[1]), 0.5, dtype=np.float32)
        
        # Ensure depths match image dimensions
        h, w = virtual_img.shape[:2]
        
        if virtual_depth.shape != (h, w):
            virtual_depth = cv2.resize(virtual_depth, (w, h))
        
        if physical_depth.shape != (h, w):
            physical_depth = cv2.resize(physical_depth, (w, h))
        
        # Get occlusion tolerance parameter
        tolerance = self.get_parameter('occlusion_tolerance').value
        
        # Create occlusion mask
        # Virtual objects are occluded if they're behind physical objects
        # Depth convention: smaller values = closer to camera
        occlusion_mask = virtual_depth > (physical_depth + tolerance)
        
        # Convert to uint8 mask (visible = 255, occluded = 0)
        visibility_mask = (~occlusion_mask).astype(np.uint8) * 255
        
        return visibility_mask

    def try_publish_merged(self):
        """Attempt to publish merged image if all required data is available"""
        if self.physical_image is None or self.virtual_image is None:
            return

        # Resize virtual overlay to match physical image size
        h, w = self.physical_image.shape[:2]
        virtual_resized = cv2.resize(self.virtual_image, (w, h))

        # Create transparency mask (remove black background from virtual image)
        black_threshold = self.get_parameter('black_threshold').value
        mask_near_black = cv2.inRange(
            virtual_resized, 
            (0, 0, 0), 
            (black_threshold, black_threshold, black_threshold)
        )
        transparency_mask = cv2.bitwise_not(mask_near_black)

        # Create depth-based occlusion mask
        occlusion_mask = self.create_occlusion_mask(
            virtual_resized, self.virtual_depth, self.physical_depth
        )

        # Combine transparency and occlusion masks
        # Virtual pixels are visible if they are:
        # 1. Not black (transparent background)
        # 2. Not occluded by physical objects
        combined_mask = cv2.bitwise_and(transparency_mask, occlusion_mask)

        # Apply combined mask to create final overlay
        result_image = self.physical_image.copy()
        
        # Use the mask to blend virtual content onto physical image
        mask_bool = combined_mask > 0
        result_image[mask_bool] = virtual_resized[mask_bool]

        # Add debug visualization if enabled
        if self.get_parameter('debug_mode').value:
            self.add_debug_visualization(result_image, combined_mask, occlusion_mask, transparency_mask)

        # Publish the merged result
        try:
            msg = self.bridge.cv2_to_imgmsg(result_image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.merged_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert and publish merged image: {e}')

    def add_debug_visualization(self, image, combined_mask, occlusion_mask, transparency_mask):
        """Add debug information to the image"""
        # Status text
        status_texts = []
        
        if self.physical_depth is not None:
            status_texts.append("Physical Depth: ON")
        else:
            status_texts.append("Physical Depth: OFF")
            
        if self.virtual_depth is not None:
            status_texts.append("Virtual Depth: ON")
        else:
            status_texts.append("Virtual Depth: OFF")
        
        # Count visible virtual pixels
        visible_pixels = np.sum(combined_mask > 0)
        transparency_pixels = np.sum(transparency_mask > 0)
        occluded_pixels = transparency_pixels - visible_pixels
        
        status_texts.append(f"Virtual pixels: {visible_pixels}")
        status_texts.append(f"Occluded: {occluded_pixels}")
        
        # Draw status text
        for i, text in enumerate(status_texts):
            y_pos = 25 + i * 25
            cv2.putText(image, text, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAwareImageOverlayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received interrupt signal")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()