import socket
import json
from models import ViveTrackerMessage
from triad_openvr import TriadOpenVR
import time
import logging
from typing import Optional, Dict, List, Any


class ViveTrackerPublisher:
    def __init__(self, host: str, port: int, interval: float = 1 / 250, max_retry: int = 10000):
        self.host = host
        self.port = port
        self.interval = interval
        self.max_retry = max_retry
        self.s: Optional[socket.socket] = None
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.logger = logging.getLogger("Vive Tracker Publisher")
        self.initialize_openvr()

    def initialize_openvr(self):
        try:
            self.triad_openvr = TriadOpenVR()
            self.triad_openvr.print_discovered_objects()
        except Exception as e:
            self.logger.error(f"Failed to Initialize Socket. Make sure subscriber is running. Error: {e}")

    def initialize_socket(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.connect((self.host, self.port))
        except ConnectionRefusedError as e:
            self.logger.error("Please make sure Vive Tracker Subscriber is running")
            self.s = None
        except Exception as e:
            self.logger.error(f"Unknown error happened: {e}")
            exit(1)

    def poll(self) -> List[ViveTrackerMessage]:
        trackers = self.get_trackers()
        messages:List[ViveTrackerMessage] = []
        for tracker_name, tracker in trackers.items():
            euler = tracker.get_pose_euler()
            x, y, z, yaw, pitch, roll = euler
            message = ViveTrackerMessage(valid=True, x=x, y=y, z=z,
                                         yaw=yaw, pitch=pitch, roll=roll, device_name=tracker_name)
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
            except ConnectionResetError as e:
                self.logger.error("Client Disconnected")
                self.s = None

            sleep_time = self.interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def send_message(self, data: ViveTrackerMessage):
        m = f"Sending: {data}"
        self.logger.info(m)
        data_to_send = self.construct_json_message(data=data)
        if self.s is not None:
            self.s.sendall(data_to_send.encode())
        else:
            self.logger.error("Unable to send message: Socket not initalized")
            self.initialize_socket()

    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    vive_tracker_publisher = ViveTrackerPublisher(host="127.0.0.1", port=8000)
    vive_tracker_publisher.start()
