#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import atexit
import time
import math
import json
import os
from mr_pkg.modules.telemetry import Telemetry
from ament_index_python.packages import get_package_share_directory

class AutowareVehicle(Node):
    def __init__(self, agent_id):
        super().__init__('Autoware_Vehicle_Telemetry')

        self.ego_vehicle_odom_subscriber = self.create_subscription(
            Odometry, 
            '/world_transform', 
            self.on_ego_vehicle_odom_received,
            10)

        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0

        self.autoware_vehicle_telemetry = AutowareVehicleTelemetry(self)
        self.autoware_vehicle_telemetry.start()
        atexit.register(self.autoware_vehicle_telemetry.shutdown)

    def on_ego_vehicle_odom_received(self, msg: Odometry):

        def set_position():
            self.x = msg.pose.pose.position.x * 100.0,    # Forward (meters to cm)
            self.y = -msg.pose.pose.position.y * 100.0,   
            self.z = msg.pose.pose.position * 100.0  

        def set_orientation():   
            # Yaw extraction from quaternion
            x = msg.pose.pose.orientationx
            y = msg.pose.pose.orientation.y  
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w
            self.yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

        # UPDATE TRANSFORMS
        set_position()
        set_orientation()

    def shutdown(self):
        self.autoware_vehicle_telemetry.shutdown()


class AutowareVehicleTelemetry(Telemetry):
    """Template class that inherits Telemetry class and provides bare minimum functionality.
    Modify this class to add your own functionality. """

    def __init__(self, aw_vehicle):
        super().__init__()

        self._is_running = False
        self.vehicles = {}
        self.aw_vehicle = aw_vehicle

    def on_receive_telemetry(self, parsed_message):
        """ Callback function to handle telemetry messages, you'll receive a dictionary with the telemetry data
        from other agents. """
        pass

    def handle_fetch_telemetry_data(self):
        """ Callback function to fetch the telemetry data to be sent to other agents.
        Return a dictionary with the telemetry data. Convert to Unreal Engine Coordinates first"""
        return  {
            "location":{
                "x": self.aw_vehicle.x,
                "y": -self.aw_vehicle.y,
                "z": self.aw_vehicle.z
            },
            "yaw": self.aw_vehicle.yaw,
            "blueprint": "vehicle.lincoln.mkz_2020",
            "color": "255, 255, 255"
        }
    
    def _load_redis_config(self):
        pkg_share = get_package_share_directory('mr_pkg')
        config_path = os.path.join(pkg_share, "config", self.CONFIG_FILE)

        try:
            with open(config_path, "r") as file:
                config = json.load(file)

                self.HOST = config.get("host", self.DEFAULT_HOST)
                self.PORT = config.get("port", self.DEFAULT_PORT)
                self.PASSWORD = config.get("password", self.DEFAULT_PASSWORD)
                self.CHANNEL = config.get("channel", self.DEFAULT_CHANNEL)

        except Exception as e:
            raise RuntimeError(f"[!] Error loading telemetry config file: {e}") from e
            

    def on_receive_conn_destroy(self, agent_id):
        """ Callback function to clean up when an agent disconnects, you'll get the id of the agent that disconnected. """
        pass

    def start(self):
        """ Start the telemetry services. """
        if not self._is_running:
            self.start_telemetry_services()

            self.vehicles = {}
            self._is_running = True

    def shutdown(self):
        """ Shutdown the telemetry services. """
        if self._is_running:
            self.stop_telemetry_services()

            self._is_running = False


# ------------ Engine ------------------
def main(args=None):
    rclpy.init(args=args)
    autoware_vehicle = AutowareVehicle("UB_autoware_lincoln")
    rclpy.spin(autoware_vehicle)
    autoware_vehicle.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()





