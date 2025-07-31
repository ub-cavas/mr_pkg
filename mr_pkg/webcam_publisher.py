import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from transformers import pipeline
import threading
from queue import Queue
from PIL import Image as PILImage

class WebcamDepthPublisher(Node):
    def __init__(self):
        super().__init__('webcam_depth_publisher')
        
        # Publishers
        self.image_publisher = self.create_publisher(Image, 'webcam/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'webcam/depth', 10)
        
        # OpenCV and ROS bridge setup
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        # Parameters (declare early so they're available for model loading)
        self.declare_parameter('depth_model', 'LiheYoung/depth-anything-base-hf')
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('depth_colormap', False)  # Whether to publish depth as colormap for visualization
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open webcam.')
            rclpy.shutdown()
            return
            
        # Set camera properties for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Depth estimation setup
        self.get_logger().info("Loading depth estimation model...")
        
        # Declare parameter for model selection
        model_name = self.get_parameter('depth_model').value
        
        try:
            # Available models (you can change via parameter):
            # - 'LiheYoung/depth-anything-base-hf' (recommended, fast and accurate)
            # - 'Intel/dpt-large' (classic, larger but good quality)
            # - 'depth-anything/Depth-Anything-V2-base-hf' (latest version)
            # - 'Intel/zoedepth-nyu-kitti' (metric depth estimation)
            
            self.depth_estimator = pipeline(
                'depth-estimation', 
                model=model_name,
                device=0 if torch.cuda.is_available() else -1
            )
            self.get_logger().info(f"Depth estimation model '{model_name}' loaded on {'GPU' if torch.cuda.is_available() else 'CPU'}")
        except Exception as e:
            self.get_logger().error(f"Failed to load depth model '{model_name}': {e}")
            self.get_logger().info("Available models: LiheYoung/depth-anything-base-hf, Intel/dpt-large, depth-anything/Depth-Anything-V2-base-hf")
            self.depth_estimator = None
        
        # Depth processing queue and thread
        self.depth_queue = Queue(maxsize=3)  # Small queue to avoid memory buildup
        self.latest_depth = None
        self.depth_lock = threading.Lock()
        
        # Start depth processing thread
        if self.depth_estimator is not None:
            self.depth_thread = threading.Thread(target=self._depth_processing_worker, daemon=True)
            self.depth_thread.start()
            self.get_logger().info("Depth processing thread started")
        
        # Timer for image capture and publishing
        self.timer = self.create_timer(0.0333, self.timer_callback)  # 30 Hz

    def _depth_processing_worker(self):
        """Worker thread for depth estimation"""
        while rclpy.ok():
            try:
                # Get image from queue (blocking)
                frame = self.depth_queue.get(timeout=1.0)
                
                if frame is None:
                    continue
                
                # Convert BGR to RGB and then to PIL Image for the depth model
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                pil_image = PILImage.fromarray(rgb_frame)
                
                # Estimate depth
                depth_result = self.depth_estimator(pil_image)
                depth_array = np.array(depth_result['depth'])
                
                # Normalize depth to 0-1 range
                depth_min = depth_array.min()
                depth_max = depth_array.max()
                if depth_max > depth_min:  # Avoid division by zero
                    depth_normalized = (depth_array - depth_min) / (depth_max - depth_min)
                else:
                    depth_normalized = np.zeros_like(depth_array)
                
                # Store the latest depth result
                with self.depth_lock:
                    self.latest_depth = depth_normalized
                
                # Log successful depth estimation occasionally 
                if hasattr(self, '_depth_success_count'):
                    self._depth_success_count += 1
                else:
                    self._depth_success_count = 1
                
                # Log every 30 successful depth estimations (about once per second at 30fps)
                if self._depth_success_count % 30 == 1:
                    self.get_logger().info(
                        f"Depth estimation working: {depth_normalized.shape}, "
                        f"range [{depth_normalized.min():.3f}, {depth_normalized.max():.3f}]"
                    )
                
                self.depth_queue.task_done()
                
            except Exception as e:
                if "Empty" not in str(e):  # Don't log timeout errors
                    self.get_logger().error(f'Depth processing error: {e}')
                    # Log more details for debugging
                    if 'frame' in locals():
                        self.get_logger().error(f'Frame shape: {frame.shape if frame is not None else "None"}')
                    if 'pil_image' in locals():
                        self.get_logger().error(f'PIL image size: {pil_image.size if pil_image is not None else "None"}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from webcam.')
            return
        
        # Always publish the image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {e}')
            return
        
        # Add frame to depth processing queue (non-blocking)
        if (self.depth_estimator is not None and 
            self.get_parameter('publish_depth').value and 
            not self.depth_queue.full()):
            try:
                self.depth_queue.put_nowait(frame.copy())
            except:
                pass  # Queue full, skip this frame
        
        # Publish depth if available
        if self.get_parameter('publish_depth').value:
            self.publish_depth()

    def publish_depth(self):
        """Publish the latest depth estimation"""
        with self.depth_lock:
            if self.latest_depth is None:
                return
            
            depth_to_publish = self.latest_depth.copy()
        
        try:
            if self.get_parameter('depth_colormap').value:
                # Convert to colormap for visualization
                depth_colormap = cv2.applyColorMap(
                    (depth_to_publish * 255).astype(np.uint8), 
                    cv2.COLORMAP_JET
                )
                depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
            else:
                # Publish as single-channel float32
                depth_msg = self.bridge.cv2_to_imgmsg(
                    depth_to_publish.astype(np.float32), 
                    encoding='32FC1'
                )
            
            self.depth_publisher.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish depth: {e}')

    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info("Shutting down webcam depth publisher...")
        
        # Signal depth thread to stop
        if hasattr(self, 'depth_queue'):
            self.depth_queue.put(None)  # Poison pill
        
        # Release camera
        if hasattr(self, 'cap'):
            self.cap.release()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamDepthPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received interrupt signal")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()