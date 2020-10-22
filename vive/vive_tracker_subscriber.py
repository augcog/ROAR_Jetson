from ROAR_Jetson.vive.models import ViveTrackerMessage
import socket
import json
import logging
from typing import Optional


class ViveTrackerSubscriber:
    def __init__(self, host: str, port: int, tracker_name: str):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.latest_tracker_message: Optional[ViveTrackerMessage] = None
        self.logger = logging.getLogger("Vive Tracker Subscriber")
        self.logger.debug("Initialization Success")

    def update(self):
        pass

    def run_threaded(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        data = ""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            while True:
                conn, addr = s.accept()
                while True:  # Accept multiple messages from each client
                    buffer = conn.recv(1)
                    buffer = buffer.decode()
                    if buffer == ";":
                        conn.close()
                        d = json.loads(json.loads(data))
                        data = ""
                        vive_tracker_message = ViveTrackerMessage.parse_obj(d)
                        self.latest_tracker_message = vive_tracker_message
                        break
                    elif buffer:
                        data += buffer
                    else:
                        break
                print(self.latest_tracker_message)
