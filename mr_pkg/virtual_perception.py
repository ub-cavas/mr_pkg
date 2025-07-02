import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray

class VirtualPerception(Node):
    def __init__(self):
        super().__init__('Virtual_Perception')
        self.virtual_object_subscriber = self.create_subscription(
            BoundingBox3DArray,
            '/virtual_obstacles',
            self.on_virtual_obstacles_recieved,
            10)
        
    def on_virtual_obstacles_recieved(self, msg: BoundingBox3DArray):
        count = len(msg.boxes)
        print("Seeing ", count, " boxes" )
        

# ------------ Engine ------------------
def main(args=None):
    print("Starting Virtual Perception Node")
    rclpy.init(args=args)
    world_transformation = VirtualPerception()
    rclpy.spin(world_transformation)
    world_transformation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()