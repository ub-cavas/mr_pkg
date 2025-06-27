import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry



# WGS84 ellipsoid constants
a = 6378137.0                             # semi‐major axis in meters
f = 1/298.257223563                       # flattening
e_sq = f * (2 - f)                        # square of eccentricity

class WorldTransformation(Node):
    def __init__(self):
        super().__init__('World_Transformation')
        self.world_origin_gnss = None
        self.ego_vehicle_gnss = None
        self.ego_vehicle_odom = Odometry()
        self.ego_vehicle_world = Odometry()

        # TODO: convert this into a service instead of a sub
        self.world_origin_subscriber = self.create_subscription(
            NavSatFix,
            'map_origin',
            self.on_world_origin_received,
            10)
        
        self.ego_vehicle_subscriber = self.create_subscription(
            NavSatFix, 
            '/vehicle/gps/fix', 
            self.on_ego_vehicle_nav_received,
            10)
        
        self.ego_vehicle_odom_subscriber = self.create_subscription(
            Odometry, 
            '/novatel/oem7/odom', 
            self.on_ego_vehicle_odom_received,
            10)
        
        self.ego_vehicle_publisher = self.create_publisher(Odometry, 'world_transform', 10)
        rate = 0.02 # 50 Hz
        self.timer = self.create_timer(rate, self.send_vehicle_world_odometry)

    # ----------- ROS2 Callbacks------------------
    def on_world_origin_received(self, msg: NavSatFix):
        self.world_origin_gnss = msg

    
    def on_ego_vehicle_nav_received(self, msg: NavSatFix):
        print("Recieved Ego Vehicle GNSS!")
        self.ego_vehicle_gnss = msg
        if self.world_origin_gnss is not None:
            x,y,z = gnss_to_world(msg.latitude, msg.longitude, msg.altitude, self.world_origin_gnss.latitude, self.world_origin_gnss.longitude, self.world_origin_gnss.altitude)
            # World Position
            self.ego_vehicle_world.pose.pose.position.x = x
            self.ego_vehicle_world.pose.pose.position.y = y
            self.ego_vehicle_world.pose.pose.position.z = z


    def on_ego_vehicle_odom_received(self, msg: Odometry):
        print("Received vehicle odometry!") 
        print(msg.header.frame_id)    
        self.ego_vehicle_odom = msg
        

    def send_vehicle_world_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'           # world frame
        msg.child_frame_id = 'base_link'       # robot frame
        # Pose
        msg.pose.pose.position = self.ego_vehicle_world.pose.pose.position
        msg.pose.pose.orientation = self.ego_vehicle_odom.pose.pose.orientation
        # Twist
        msg.twist.twist.linear = self.ego_vehicle_odom.twist.twist.linear
        msg.twist.twist.angular = self.ego_vehicle_odom.twist.twist.angular
        self.ego_vehicle_publisher.publish(msg)
         
# ----------- Conversions ------------------

def geodetic_to_ecef(lat, lon, h):
    """
    Convert geodetic coords (degrees, meters) to ECEF (X, Y, Z) in meters.
    """
    phi = math.radians(lat)
    lam = math.radians(lon)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    N = a / math.sqrt(1 - e_sq * sin_phi**2)
    X = (N + h) * cos_phi * math.cos(lam)
    Y = (N + h) * cos_phi * math.sin(lam)
    Z = (N * (1 - e_sq) + h) * sin_phi
    return X, Y, Z

def enu_offset(lat0, lon0, h0, lat1, lon1, h1):
    """
    Compute the E, N, Up offset from P0=(lat0,lon0,h0) to P1=(lat1,lon1,h1).
    Returns (east, north, up) in meters.
    """
    # 1) ECEF of each point
    X0, Y0, Z0 = geodetic_to_ecef(lat0, lon0, h0)
    X1, Y1, Z1 = geodetic_to_ecef(lat1, lon1, h1)

    # 2) Vector difference in ECEF
    dX, dY, dZ = X1 - X0, Y1 - Y0, Z1 - Z0

    # 3) Precompute sines/cosines of origin lat/lon
    phi = math.radians(lat0)
    lam = math.radians(lon0)
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    sin_lam, cos_lam = math.sin(lam), math.cos(lam)

    # 4) Rotate into local ENU
    east  = -sin_lam * dX + cos_lam * dY
    north = -cos_lam * sin_phi * dX \
            - sin_phi * sin_lam * dY \
            + cos_phi * dZ
    up    =  cos_phi * cos_lam * dX \
            + cos_phi * sin_lam * dY \
            + sin_phi * dZ

    return east, up, north

# Using the map origin, convert a gnss pose to a world pose (meters)
def gnss_to_world(lat: float, lon: float, alt: float, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
        """
        Convert a GNSS fix (lat, lon, alt) to a Unity world position
        relative to the provided map origin (origin_lat, origin_lon, origin_alt).

        Unity axes convention:
        X → East
        Y → Up
        Z → North

        Returns a tuple (x, y, z) in meters.
        """
        # 1) GNSS → ECEF
        return enu_offset(lat,lon,alt,origin_lat,origin_lon,origin_alt)
           

def world_to_gnss(x: float, y: float, z: float, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
    pass

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