import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DualEkfLocalization(Node):
    def __init__(self):
        super().__init__('dual_ekf_world_transform')
        
        self.ego_vehicle_odom_subscriber = self.create_subscription(
            Odometry, 
            '/odometry/global', 
            self.on_ego_vehicle_odom_received,
            10)
        
        self.ego_vehicle_publisher = self.create_publisher(Odometry, '/ub_mr/localization', 10)

    # ----------- ROS2 Callbacks------------------
    def on_ego_vehicle_odom_received(self, msg: Odometry):
        newMsg = Odometry()
        newMsg.header.stamp = self.get_clock().now().to_msg()
        newMsg.header.frame_id = 'odom'           # world frame
        newMsg.child_frame_id = 'base_link'       # robot frame   
        newMsg.pose.pose = msg.pose.pose
        self.ego_vehicle_publisher.publish(newMsg)
    

# ------------ Engine ------------------
def main(args=None):
    print("Starting Dual EKF Localization Node")
    rclpy.init(args=args)
    dual_ekf_localization = DualEkfLocalization()
    rclpy.spin(dual_ekf_localization)
    dual_ekf_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
