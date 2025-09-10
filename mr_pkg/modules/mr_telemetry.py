import socket
import json
import atexit
import time
from ament_index_python.packages import get_package_share_directory
from mr_pkg.modules.telemetry import Telemetry
import os

class TrafficAgent:
    def __init__(self, id, location, yaw, blueprint, color):
        self.id = id
        self.position = location
        self.yaw = yaw
        self.blueprint = blueprint
        self.color = color

class MrTelemetry(Telemetry):
    def __init__(self):
            super().__init__()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.address = ('127.0.0.1', 12345)
            self._is_running = False
            self.vehicles = {}

    def on_receive_telemetry(self, parsed_message):
        id = parsed_message["id"]
        bp = parsed_message["blueprint"]
        color = parsed_message["color"]
        position = parsed_message["location"]
        yaw = parsed_message["yaw"]

        if id not in self.vehicles:
            self.vehicles[id] = TrafficAgent(id, position, yaw, bp, color).__dict__
        elif self._has_other_vehicle_changed(id, bp, color):
            del self.vehicles[id]
            self.vehicles[id] = TrafficAgent(id, position, yaw, bp, color).__dict__
        else:
            self.vehicles[id].location = position
            self.vehicles[id].yaw = yaw
        
        self.send_traffic_data(self.vehicles)

    def _has_other_vehicle_changed(self, id, bp, color):
        vehicle = self.vehicles[id]
        return vehicle.blueprint != bp or vehicle.attributes["color"] != color

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
        # TODO: Send a message to Unity to remove the vehicle
        pass  

    def pack_vehicle_message(id, location, yaw):
        """ Convert from Unreal Engine coordinates to Unity coordinates. """
        return {
            "x": location["x"],
            "y": location["z"],
            "z": -location["y"],
            "yaw": -yaw + 90
        }
    
    def pack_new_vehicle_message(id, location, yaw, blueprint, color):
        vehicle_msg = self.pack_vehicle_message(id, location, yaw)
        vehicle_msg["blueprint"] = blueprint
        vehicle_msg["color"] = color
        vehicle_msg["id"] = id
        vehicle_msg["is_new"] = True
        return vehicle_msg

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

    def send_traffic_data(self, traffic_dict):
        try:
            # Serialize dictionary to JSON
            json_data = json.dumps(traffic_dict)
            # Send as bytes
            self.sock.sendto(json_data.encode('utf-8'), self.address)
        except Exception as e:
            print(f"Error sending data: {e}")

    def close(self):
        self.sock.close()

# -------------Example usage------------------
mr_telemetry = MrTelemetry()
mr_telemetry.start()
atexit.register(mr_telemetry.shutdown)