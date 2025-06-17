#mr_pkg/mr_pkg/world_transformation.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from math import sin, cos, sqrt, radians, atan2, degrees

# WGS84 ellipsoid constants
_A = 6378137.0             # semi-major axis, meters
_E2 = 6.69437999014e-3     # first eccentricity squared
_B    = _A * sqrt(1 - _E2)    # semi-minor axis
_EP2  = (_A**2 - _B**2) / _B**2  # second eccentricity squared

class WorldTransformation(Node):
    def __init__(self):
        super().__init__('world_transformation')
        self.world_origin_gnss = None
        self.ego_vehicle_gnss = None
        self.ego_vehicle_world = Vector3()

        # TODO: convert this into a service instead of a sub
        self.world_origin_subscriber = self.create_subscription(
            NavSatFix,
            'map_origin',
            self.on_world_origin_received,
            10)
        
        self.ego_vehicle_subscriber = self.create_subscription(
            NavSatFix, 
            '/vehicle/gps/fix', 
            self.on_ego_vehicle_received,
            10)
        
        self.ego_vehicle_publisher = self.create_publisher(Odometry, 'ego_vehicle_world', 10)
        rate = 0.05 # 20 Hz
        self.timer = self.create_timer(rate, self.send_vehicle_world_odometry)

    # ----------- ROS2 Callbacks------------------
    def on_world_origin_received(self, msg: NavSatFix):
        print("Recieved world origin!")
        self.world_origin_gnss = msg

    
    def on_ego_vehicle_received(self, msg: NavSatFix):
        print("Recieved Ego Vehicle GNSS!")
        self.ego_vehicle_gnss = msg
        if self.world_origin_gnss is not None:
            self.ego_vehicle_world.x, self.ego_vehicle_world.y, self.ego_vehicle_world.z = gnss_to_world(msg.latitude, msg.longitude, msg.altitude,
                                               self.world_origin_gnss.latitude, self.world_origin_gnss.longitude, self.world_origin_gnss.altitude)

    def send_vehicle_world_odometry(self):
        #TODO: assign the odometry message
        msg = Odometry()
        self.ego_vehicle_publisher.publish(msg)
         
# ----------- Conversions ------------------

def latlon_to_ecef(lat: float, lon: float, alt: float = 0.0):
    φ = radians(lat)
    λ = radians(lon)
    N = _A / sqrt(1 - _E2 * sin(φ)**2)
    x = (N + alt) * cos(φ) * cos(λ)
    y = (N + alt) * cos(φ) * sin(λ)
    z = (N * (1 - _E2) + alt) * sin(φ)
    return x, y, z

def ecef_to_latlon(x: float, y: float, z: float):
    # From ECEF to geodetic using Bowring’s method
    p = sqrt(x*x + y*y)
    θ = atan2(z * _A, p * _B)
    sinθ, cosθ = sin(θ), cos(θ)
    lat = atan2(z + _EP2 * _B * sinθ**3,
                p - _E2 * _A * cosθ**3)
    lon = atan2(y, x)
    N = _A / sqrt(1 - _E2 * sin(lat)**2)
    alt = p / cos(lat) - N
    return degrees(lat), degrees(lon), alt

def ecef_to_enu(x: float, y: float, z: float,
                    lat0: float, lon0: float, alt0: float = 0.0):
        """
        Convert ECEF (x,y,z) to local tangent plane (ENU) 
        at reference point (lat0, lon0, alt0).
        Returns (east, north, up) in meters.
        """
        # ECEF of reference origin
        x0, y0, z0 = latlon_to_ecef(lat0, lon0, alt0)
        dx, dy, dz = x - x0, y - y0, z - z0

        φ0 = radians(lat0)
        λ0 = radians(lon0)
        sinφ, cosφ = sin(φ0), cos(φ0)
        sinλ, cosλ = sin(λ0), cos(λ0)

        # East
        east  = -sinλ * dx + cosλ * dy
        # North
        north = -sinφ * cosλ * dx - sinφ * sinλ * dy + cosφ * dz
        # Up
        up    =  cosφ * cosλ * dx + cosφ * sinλ * dy + sinφ * dz

        return east, north, up

def enu_to_ecef(e: float, n: float, u: float,
                lat0: float, lon0: float, alt0: float = 0.0):
    """
    Convert local ENU vector at (lat0, lon0, alt0) back to ECEF coords.
    """
    φ0 = radians(lat0)
    λ0 = radians(lon0)
    sinφ, cosφ = sin(φ0), cos(φ0)
    sinλ, cosλ = sin(λ0), cos(λ0)

    # Rotation transpose * [e,n,u]:
    dx = -sinλ    * e + -sinφ*cosλ * n + cosφ*cosλ * u
    dy =  cosλ    * e + -sinφ*sinλ * n + cosφ*sinλ * u
    dz =      0   * e +  cosφ      * n + sinφ      * u

    x0, y0, z0 = latlon_to_ecef(lat0, lon0, alt0)
    return x0 + dx, y0 + dy, z0 + dz

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
        x, y, z = latlon_to_ecef(lat, lon, alt)
        # 2) ECEF → ENU at origin
        east, north, up = ecef_to_enu(x, y, z, origin_lat, origin_lon, origin_alt)
        # 3) Map ENU → Unity (x, y, z)
        return east, up, north       

def world_to_gnss(x: float, y: float, z: float, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
    """
    Convert Unity world-space (X, Y, Z) back to (lat, lon, alt).
     – Unity: X→East, Y→Up, Z→North
     – ENU:    e=X,   n=Z,     u=Y
    """
    e, n, u = x, z, y
    xe, ye, ze = enu_to_ecef(e, n, u, origin_lat, origin_lon, origin_alt)
    return ecef_to_latlon(xe, ye, ze)

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