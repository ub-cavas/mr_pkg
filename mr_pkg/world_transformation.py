#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from robot_localization.srv import FromLL, ToLL
from tf2_ros import TransformBroadcaster
import numpy as np

class WorldTransformation(Node):
    def __init__(self):
        super().__init__('world_transformation')
        
        # Publishers for Unity
        self.unity_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/unity/vehicle_pose', 
            10
        )
        
        # Service clients for coordinate conversion
        self.fromLL_client = self.create_client(FromLL, '/fromLL')
        self.toLL_client = self.create_client(ToLL, '/toLL')
        
        # Subscribe to the global EKF output
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odometry_callback,
            10
        )
        
        # Transform broadcaster for Unity
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Wait for services
        while not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /fromLL service...')
        
        self.get_logger().info('World transformation node initialized')
    
    def odometry_callback(self, msg: Odometry):
        """Convert robot_localization output to Unity coordinates"""
        
        # robot_localization uses ENU (East-North-Up)
        # Unity uses Y-up, Z-forward, X-right
        
        # Create Unity pose message
        unity_pose = PoseWithCovarianceStamped()
        unity_pose.header = msg.header
        unity_pose.header.frame_id = "unity_world"
        
        # Transform from ENU to Unity coordinates
        # ENU: X=East, Y=North, Z=Up
        # Unity: X=Right(East), Y=Up(Up), Z=Forward(North)
        unity_pose.pose.pose.position.x = msg.pose.pose.position.x
        unity_pose.pose.pose.position.y = msg.pose.pose.position.z  # Up
        unity_pose.pose.pose.position.z = msg.pose.pose.position.y  # Forward
        
        # Copy orientation (may need adjustment based on vehicle frame)
        unity_pose.pose.pose.orientation = msg.pose.pose.orientation
        
        # Copy covariance
        unity_pose.pose.covariance = msg.pose.covariance
        
        # Publish for Unity
        self.unity_pose_pub.publish(unity_pose)
        
        # Also broadcast as TF for visualization
        self.broadcast_unity_transform(unity_pose)
    
    def broadcast_unity_transform(self, pose_msg):
        """Broadcast transform for debugging/visualization"""
        t = TransformStamped()
        t.header = pose_msg.header
        t.child_frame_id = "vehicle_unity"
        t.transform.translation.x = pose_msg.pose.pose.position.x
        t.transform.translation.y = pose_msg.pose.pose.position.y
        t.transform.translation.z = pose_msg.pose.pose.position.z
        t.transform.rotation = pose_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    async def gps_to_map(self, lat, lon, alt=0.0):
        """Convert GPS coordinates to map frame using service"""
        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = alt
        
        future = self.fromLL_client.call_async(request)
        response = await future
        
        if response:
            return (response.map_point.x, 
                    response.map_point.y, 
                    response.map_point.z)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = WorldTransformation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()