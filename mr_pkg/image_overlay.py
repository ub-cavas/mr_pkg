#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageOverlayNode(Node):
    def __init__(self):
        super().__init__('image_overlay_node')
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Store the latest compressed image
        self.latest_compressed_image = None
        
        # Mask color: R=255 G=163 B=0 (in BGR format for OpenCV: B=0 G=163 R=255)
        bright_pink_bgr = np.array([147, 20, 255])  # BGR format
        # For masking, you typically want a range:
        lower_pink = np.array([140, 10, 240])  # Lower bound
        upper_pink = np.array([154, 30, 255])  # Upper bound

        self.mask_color_bgr = bright_pink_bgr
        # Tolerance for color matching (allows for slight variations)
        self.color_tolerance = 180
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            '/virtual_camera/image_raw/compressed',
            self.compressed_image_callback,
            10
        )
        
        # Publisher
        self.merged_image_pub = self.create_publisher(
            Image,
            '/merged_image',
            10
        )
        
        self.get_logger().info('Image Overlay Node initialized')
        self.get_logger().info(f'Mask color (BGR): {self.mask_color_bgr} with tolerance: {self.color_tolerance}')
    
    def compressed_image_callback(self, msg):
        """Store the latest compressed image"""
        try:
            # Convert compressed image to OpenCV format
            self.latest_compressed_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().debug('Received new compressed image')
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')
    
    def image_callback(self, msg):
        """Process base image and overlay compressed image if available"""
        try:
            # Convert base image to OpenCV format
            base_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # If no compressed image is available yet, just publish the base image
            if self.latest_compressed_image is None:
                self.get_logger().debug('No compressed image available, publishing base image only')
                merged_msg = self.bridge.cv2_to_imgmsg(base_image, 'bgr8')
                merged_msg.header = msg.header
                self.merged_image_pub.publish(merged_msg)
                return
            
            # Get dimensions of base image
            base_height, base_width = base_image.shape[:2]
            
            # Scale compressed image to match base image dimensions
            scaled_compressed = cv2.resize(
                self.latest_compressed_image, 
                (base_width, base_height), 
                interpolation=cv2.INTER_LINEAR
            )
            
            # Create overlay image starting with base image
            merged_image = base_image.copy()
            
            # Create mask for pixels that match the specified color (R=255 G=163 B=0)
            # Calculate color distance for each pixel
            color_diff = np.abs(scaled_compressed.astype(np.float32) - self.mask_color_bgr.astype(np.float32))
            color_distance = np.sqrt(np.sum(color_diff ** 2, axis=2))
            
            # Pixels within tolerance of the mask color should be transparent
            mask_pixels = color_distance <= self.color_tolerance
            
            # Invert mask - we want to overlay pixels that are NOT the mask color
            overlay_mask = ~mask_pixels
            
            # Apply overlay where mask is True (non-black pixels)
            merged_image[overlay_mask] = scaled_compressed[overlay_mask]
            
            # Convert back to ROS Image message
            merged_msg = self.bridge.cv2_to_imgmsg(merged_image, 'bgr8')
            merged_msg.header = msg.header  # Preserve timestamp and frame_id
            
            # Publish merged image
            self.merged_image_pub.publish(merged_msg)
            
            self.get_logger().debug('Published merged image')
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImageOverlayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()