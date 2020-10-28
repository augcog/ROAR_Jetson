import socket
import json
from models import ViveTrackerMessage
from triad_openvr import TriadOpenVR
import time
import logging
from typing import Optional, Dict, List, Any, Tuple


class ViveTrackerPublisher:
    def __init__(self, addresses: List[Tuple[str, int]], interval: float = 1 / 250, max_retry: int = 10000):
        self.interval = interval
        self.max_retry = max_retry
        self.sockets: List[socket.socket] = []
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.logger = logging.getLogger("Vive Tracker Publisher")
        self.initialize_openvr()
        self.initialize_sockets(addresses)

    def initialize_openvr(self):
        try:
            self.triad_openvr = TriadOpenVR()
            self.triad_openvr.print_discovered_objects()
        except Exception as e:
            self.logger.error(f"Failed to Initialize Socket. Make sure subscriber is running. Error: {e}")

    def initialize_sockets(self, addresses: List[Tuple[str, int]]):
        for address in addresses:
            host, port = address
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect((host, port))
                self.sockets.append(s)
            except ConnectionRefusedError as e:
                self.logger.error("Please make sure Vive Tracker Subscriber is running")
            except Exception as e:
                self.logger.error(f"Unknown error happened: {e}")
                exit(1)

    def poll(self) -> List[ViveTrackerMessage]:
        trackers = self.get_trackers()
        messages: List[ViveTrackerMessage] = []
        for tracker_name, tracker in trackers.items():
            euler = tracker.get_pose_euler()
            vel_x, vel_y, vel_z = tracker.get_velocity()
            # print(vel_x, vel_y, vel_z)
            x, y, z, yaw, pitch, roll = euler
            message = ViveTrackerMessage(valid=True, x=x, y=y, z=z,
                                         yaw=yaw, pitch=pitch, roll=roll,
                                         vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
                                         device_name=tracker_name)
            messages.append(message)
        return messages

    def get_trackers(self) -> Dict[str, Any]:
        trackers = {tracker_name: tracker for tracker_name, tracker in self.triad_openvr.devices.items()
                    if "tracker" in tracker_name}
        return trackers

    def start(self):
        error_count = 0
        self.logger.info("Tracking Started")
        while error_count < self.max_retry:
            start = time.time()
            try:
                messages = self.poll()
                for message in messages:
                    self.send_message(data=message)
                error_count = 0
            except TypeError:
                self.logger.error(f"Unable to Connect to Vive Tracker, trying again {error_count}. "
                                  f"Try Moving the Tracker to reactivate it")
                error_count += 1
            except ConnectionAbortedError:
                self.logger.error("Failed to send")

            sleep_time = self.interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def send_message(self, data: ViveTrackerMessage):
        m = f"Sending: {data}"
        self.logger.info(m)
        data_to_send = self.construct_json_message(data=data)
        for socket in self.sockets:
            socket.sendall(data_to_send.encode())

    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    vive_tracker_publisher = ViveTrackerPublisher(addresses=[("192.168.1.7", 8000)])
    vive_tracker_publisher.start()
