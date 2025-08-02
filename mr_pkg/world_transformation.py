import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose

class WorldTransformation(Node):
    def __init__(self):
        super().__init__('World_Transformation')
        
        self.ego_vehicle_odom_subscriber = self.create_subscription(
            Odometry, 
            '/odometry/global', 
            self.on_ego_vehicle_odom_received,
            10)
        
        self.ego_vehicle_publisher = self.create_publisher(Odometry, '/world_transform', 10)

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
    print("Starting World_Transform Node")
    rclpy.init(args=args)
    world_transformation = WorldTransformation()
    rclpy.spin(world_transformation)
    world_transformation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()