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
        self.location = location
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
        agent_id = parsed_message["id"]
        bp = parsed_message["blueprint"]
        color = parsed_message["color"]
        position = parsed_message["location"]
        yaw = parsed_message["yaw"]

        if agent_id not in self.vehicles:
            self.vehicles[agent_id] = TrafficAgent(agent_id, position, yaw, bp, color).__dict__
        elif self._has_other_vehicle_changed(agent_id, bp, color):
            del self.vehicles[agent_id]
            self.vehicles[agent_id] = TrafficAgent(agent_id, position, yaw, bp, color).__dict__
        else:
            self.vehicles[agent_id].location = position
            self.vehicles[agent_id].yaw = yaw
        
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
        del self.vehicles[agent_id]

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

    def send_traffic_data(self):
        try:
            # Serialize dictionary to JSON
            json_data = json.dumps(self.vehicles)
            # Send as bytes
            self.sock.sendto(json_data.encode('utf-8'), self.address)
        except Exception as e:
            print(f"Error sending data: {e}")

    def close(self):
        self.sock.close()

def main():
    x = 0
    mr_telemetry = MrTelemetry()
    mr_telemetry.start()
    atexit.register(mr_telemetry.shutdown)
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("[x] Keyboard interrupt")

    finally:
        mr_telemetry.shutdown()

if __name__ == '__main__':
    main()

# -------------Example usage------------------
    """while (True):
        mr_telemetry.vehicles = {
            "agent_001": {
                "location": {"x": 10.0 + x, "y": 0.0, "z": 5.0},
                "yaw": 90.0,
                "blueprint": "vehicle.lincoln.mkz_2020",
                "color": "255, 0, 0"
            },
            "agent_002": {
                "location": {"x": -5.0 + x, "y": 0.0, "z": 10.0},
                "yaw": 180.0,
                "blueprint": "vehicle.tesla.model3",
                "color": "0, 255, 0"
            },
            "agent_003": {
                "location": {"x": 0.0 + x, "y": 0.0, "z": -8.0},
                "yaw": 45.0,
                "blueprint": "vehicle.bmw.grandtourer",
                "color": "0, 0, 255"
            }
        }
        mr_telemetry.send_traffic_data()
        print("Sent telemetry data")
        time.sleep(0.1)
        x += 0.05"""


