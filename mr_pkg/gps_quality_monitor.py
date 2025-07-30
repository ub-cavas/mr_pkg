import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

class GPSQualityMonitor(Node):
    def __init__(self):
        super().__init__('gps_quality_monitor')
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/vehicle/gps/fix',
            self.gps_callback,
            10
        )
        
        # Publish GPS quality for Unity visualization
        self.quality_pub = self.create_publisher(
            Float32,
            '/unity/gps_quality',
            10
        )
    
    def gps_callback(self, msg: NavSatFix):
        # Check GPS fix type
        fix_type = msg.status.status
        
        # NavSatStatus values:
        # -1 = NO_FIX, 0 = FIX, 1 = SBAS_FIX, 2 = GBAS_FIX
        
        # Check covariance for RTK quality
        h_accuracy = np.sqrt(msg.position_covariance[0] + 
                            msg.position_covariance[4])
        
        quality_msg = Float32()
        
        if fix_type == 2:  # RTK Fixed
            quality_msg.data = 1.0  # Highest quality
            self.get_logger().info('RTK Fixed - cm accuracy')
        elif fix_type == 1:  # RTK Float/DGPS
            quality_msg.data = 0.7
            self.get_logger().info('RTK Float - dm accuracy')
        elif fix_type == 0 and h_accuracy < 2.0:  # Good GPS
            quality_msg.data = 0.5
            self.get_logger().info('Standard GPS - Good')
        else:  # Poor GPS
            quality_msg.data = 0.2
            self.get_logger().warn('Poor GPS quality')
        
        self.quality_pub.publish(quality_msg)