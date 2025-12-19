import socket
import json
import atexit
import time
import threading
import os

from ament_index_python.packages import get_package_share_directory
from mr_pkg.modules.telemetry import Telemetry

import carla

CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 10.0

class TrafficAgent:
    def __init__(self, id, location, yaw, blueprint, color):
        self.id = id
        self.location = location
        self.yaw = yaw
        self.blueprint = blueprint
        self.color = color
    
    def to_dict(self):
        return{
            "id": self.id,
            "location": self.location,
            "yaw": self.yaw,
            "blueprint": self.blueprint,
            "color": self.color
        }

class MrTelemetry(Telemetry):
    def __init__(self):
            super().__init__()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.address = ('127.0.0.1', 12345)
            self._is_running = False
            self.vehicles = {}

            self._vehicles_lock = threading.Lock() #Lock to avoid race

            self._connect_to_carla()

    def _connect_to_carla(self):
        if carla is None:
            print('Module carla not found')
        try:
            client = carla.Client(CARLA_HOST, CARLA_PORT)
            client.set_timeout(CARLA_TIMEOUT)
            self.carla_client = client
            self.carla_world = client.get_world()
        except Exception as e:
            self.carla_client = None
            self.carla_world = None
            print('Failed to connect to the simulator. Make sure the simulator is running and connected on localhost:2000')

    def _collect_carla_traffic_data(self):
        if self.carla_world is None: #If world is None, attempt to reconnect
            self._connect_to_carla()
            if self.carla_world is None:
                return []
        
        try:
            world = self.carla_world
            actors = world.get_actors()
            vehicles = actors.filter("vehicle.*")
            walkers = actors.filter("walker.pedestrian.*")

            items = []

            for vehicle in vehicles:
                try:
                    bbox_rel = vehicle.bounding_box
                    t = vehicle.get_transform()
                    center_world = t.transform(bbox_rel.location)

                    items.append({
                        "actor_id": vehicle.id,
                        "actor_type": "vehicle",
                        "type_id": vehicle.type_id,
                        "location": {"x": center_world.x, "y": center_world.y, "z": center_world.z},
                        "rotation": {"roll": t.rotation.roll, "pitch": t.rotation.pitch, "yaw": t.rotation.yaw},
                        "extent": {"x": bbox_rel.extent.x, "y": bbox_rel.extent.y, "z": bbox_rel.extent.z},
                        "velocity": {"x": vehicle.get_velocity().x, "y": vehicle.get_velocity().y, "z": vehicle.get_velocity().z}
                    })
                except Exception:
                    print("Failed to collect CARLA vehicle traffic data")

            return items
        except Exception:
            print("Failed to collect CARLA traffic data")
            return []


    def on_receive_telemetry(self, parsed_message):
        agent_id = parsed_message.get("id")
        bp = parsed_message.get("blueprint")
        color = parsed_message.get("color")
        position = parsed_message.get("location")
        yaw = parsed_message.get("yaw")

        with self._vehicles_lock:
            if agent_id not in self.vehicles:
                self.vehicles[agent_id] = {
                   "location": position,
                    "yaw": yaw,
                    "blueprint": bp,
                    "color": color 
                }
            else:
                vehicle = self.vehicles[agent_id]
                changed = (vehicle.get("blueprint") != bp) or (vehicle.get("color") != color)
                if changed:
                    self.vehicles[agent_id] = {
                        "location": position,
                        "yaw": yaw,
                        "blueprint": bp,
                        "color": color
                    }
                else:
                    vehicle["location"] = position
                    vehicle["yaw"] = yaw

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

    def handle_fetch_telemetry_data(self):
        with self._vehicles_lock: #Grab lock to avoid race
            vehicles_snapshot = dict(self.vehicles)
        
        try:
            traffic = self._collect_carla_traffic_data()
        except Exception:
            traffic = []
            print("_collect_carla_actors failed")
        
        payload = {
            "vehicles": vehicles_snapshot,
            "traffic": traffic
        }

        self.send_data_over_udp(payload)

        return payload

    def send_data_over_udp(self, payload):
        try:
            # Serialize dictionary to JSON
            json_data = json.dumps(payload)
            # Send as bytes
            self.sock.sendto(json_data.encode('utf-8'), self.address)
        except Exception as e:
            print(f"Error sending data over UDP: {e}")

    def close(self):
        self.sock.close()

def main():
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


