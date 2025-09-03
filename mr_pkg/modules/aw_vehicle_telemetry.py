import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import atexit
import time

if __name__ == "__main__":
    from telemetry import Telemetry
else:
    from modules.telemetry import Telemetry

class AutowareVehicle(Node):

    def __init__(self, agent_id):
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
        print(f"Received odom: {msg}")

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
        Return a dictionary with the telemetry data. """
        return  {
            "location":{
                "x": self.aw_vehicle.x,
                "y": self.aw_vehicle.y,
                "z": self.aw_vehicle.z
            },
            "yaw": self.aw_vehicle.yaw,
            "blueprint": hero_vehicle.attributes.get("ros_name", self.DEFAULT_BLUEPRINT),
            "color": hero_vehicle.attributes.get("color", self.DEFAULT_VEHICLE_COLOR)
        }
        pass

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
    autoware_vehicle = AutowareVehicle()
    rclpy.spin(autoware_vehicle)
    autoware_vehicle.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()





