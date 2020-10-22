import socket
import json
from models import ViveTrackerMessage
from triad_openvr import TriadOpenVR
import time
import logging
from typing import Optional


class ViveTrackerPublisher:
    def __init__(self, host: str, port: int, interval: float = 1 / 250, max_retry: int = 10000):
        self.host = host
        self.port = port
        self.interval = interval
        self.max_retry = max_retry
        self.s: Optional[socket.socket] = None
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.logger = logging.getLogger("Vive Tracker Publisher")
        self.initialize()

    def initialize(self):
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.host, self.port))
        except ConnectionRefusedError as e:
            self.logger.error("Please make sure Vive Tracker Subscriber is running")
            exit(1)
        except Exception as e:
            self.logger.error(f"Unknown error happened: {e}")
            exit(1)
        try:
            self.triad_openvr = TriadOpenVR()
            self.triad_openvr.print_discovered_objects()
        except Exception as e:
            self.logger.error("Cannot establish connection with Steam")

    def poll(self) -> ViveTrackerMessage:
        euler = self.triad_openvr.devices["tracker_1"].get_pose_euler()
        x, y, z, yaw, pitch, roll = euler
        message = ViveTrackerMessage(valid=True, x=x, y=y, z=z,
                                     yaw=yaw, pitch=pitch, roll=roll)
        return message

    def start(self):
        error_count = 0
        while error_count < self.max_retry:
            start = time.time()
            try:
                message = self.poll()
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
        self.s.sendall(data_to_send.encode())


    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data


if __name__ == "__main__":
    vive_tracker_publisher = ViveTrackerPublisher(host="127.0.0.1", port=8000)
    vive_tracker_publisher.start()
