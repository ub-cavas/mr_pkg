import socket
import json
import time

class UnityTrafficSender:
    def __init__(self, host='127.0.0.1', port=12345):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)

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
sender = UnityTrafficSender()

# Your example data
traffic_data = {
    "location": {
        "x": 0.00,
        "y": -0.00,
        "z": 0.00
    },
    "yaw": 0.00,
    "blueprint": "vehicle.lincoln.mkz_2020",
    "color": "255, 255, 255"
}

# Send at ~30 Hz
while True:
    sender.send_traffic_data(traffic_data)
    print("Sent traffic data")
    time.sleep(1/30)  # 30 Hz