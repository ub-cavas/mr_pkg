#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import struct

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        
        # Create subscriber for depth images
        self.subscription = self.create_subscription(
            Image,
            '/virtual_camera/depth',
            self.depth_callback,
            10  # Queue size
        )
        
        self.get_logger().info('Depth Image Subscriber Node Started')
        self.get_logger().info('Subscribing to: /virtual_camera/depth')
        
        # Counter to limit printing (so we don't spam the console)
        self.message_count = 0
        self.print_every = 30  # Print every 30th message (adjust as needed)

    def depth_callback(self, msg):
        try:
            # Log basic message info
            #self.get_logger().info(f'Received depth image: {msg.width}x{msg.height}, encoding: {msg.encoding}')
            
            # Verify the encoding is what we expect
            if msg.encoding != '32FC1':
                self.get_logger().warning(f'Expected 32FC1 encoding, got: {msg.encoding}')
                return
            
            # Convert byte data back to float array
            # Each pixel is 4 bytes (32-bit float)
            num_pixels = msg.width * msg.height
            expected_bytes = num_pixels * 4
            
            if len(msg.data) != expected_bytes:
                self.get_logger().error(f'Data size mismatch. Expected {expected_bytes} bytes, got {len(msg.data)}')
                return
            
            # Determine byte order based on is_bigendian flag
            endian_format = '>' if msg.is_bigendian else '<'
            
            # Unpack bytes to floats
            float_format = f'{endian_format}{num_pixels}f'
            depth_values = struct.unpack(float_format, msg.data)
            
            # Convert to numpy array and reshape to 2D
            depth_array = np.array(depth_values, dtype=np.float32)
            depth_2d = depth_array.reshape(msg.height, msg.width)
            
            # Print depth array (limit frequency to avoid spam)
            self.message_count += 1
            if self.message_count % self.print_every == 0:
                self.print_depth_info(depth_2d, msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def print_depth_info(self, depth_array, msg):
        """Print depth array information and values"""
        print("\n" + "="*80)
        print(f"DEPTH IMAGE DATA - Message #{self.message_count}")
        print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"Frame ID: {msg.header.frame_id}")
        print(f"Dimensions: {msg.width} x {msg.height}")
        print(f"Encoding: {msg.encoding}")
        print("="*80)
        
        # Print statistics
        print(f"Depth Statistics:")
        print(f"  Min depth: {np.min(depth_array):.4f} meters")
        print(f"  Max depth: {np.max(depth_array):.4f} meters")
        print(f"  Mean depth: {np.mean(depth_array):.4f} meters")
        print(f"  Std deviation: {np.std(depth_array):.4f} meters")
        
        # Print a sample of the depth array (center portion)
        print(f"\nDepth Array Sample (center 10x10 region):")
        h, w = depth_array.shape
        center_h, center_w = h // 2, w // 2
        sample_size = min(10, min(h, w))
        start_h = max(0, center_h - sample_size // 2)
        end_h = min(h, start_h + sample_size)
        start_w = max(0, center_w - sample_size // 2)
        end_w = min(w, start_w + sample_size)
        
        sample = depth_array[start_h:end_h, start_w:end_w]
        
        # Print with proper formatting
        print("Row/Col indices:", end="  ")
        for j in range(sample.shape[1]):
            print(f"{start_w + j:8d}", end="")
        print()
        
        for i in range(sample.shape[0]):
            print(f"Row {start_h + i:3d}:      ", end="")
            for j in range(sample.shape[1]):
                print(f"{sample[i, j]:8.4f}", end="")
            print()
        
        print("\n" + "="*80 + "\n")

    def print_full_array(self, depth_array):
        """Print the complete depth array - use with caution for large images!"""
        print("\nFULL DEPTH ARRAY:")
        print("="*50)
        h, w = depth_array.shape
        
        # Print column indices
        print("     ", end="")
        for j in range(w):
            print(f"{j:8d}", end="")
        print()
        
        # Print rows with row indices
        for i in range(h):
            print(f"{i:3d}: ", end="")
            for j in range(w):
                print(f"{depth_array[i, j]:8.4f}", end="")
            print()
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    
    depth_subscriber = DepthImageSubscriber()
    
    try:
        rclpy.spin(depth_subscriber)
    except KeyboardInterrupt:
        print("\nShutting down depth subscriber...")
    finally:
        depth_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()