from ROAR_Jetson.vive.models import ViveTrackerMessage
import socket
import json
import logging
from typing import Optional, List


class ViveTrackerSubscriber:
    def __init__(self, host: str, port: int, tracker_name: str, buffer_length=200):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.latest_tracker_message: Optional[ViveTrackerMessage] = None
        self.logger = logging.getLogger("Vive Tracker Subscriber")
        self.logger.debug("Initialization Success")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer_length = buffer_length

    def update(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        buffer: str = ""
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        while True:
            conn, addr = self.socket.accept()

            while True:
                received_message = conn.recv(self.buffer_length).decode()
                if received_message != '':
                    found_ending_char = ';' in received_message
                    if found_ending_char:
                        buffer = buffer + received_message[:-1]
                        self.update_latest_tracker_message(buffer)
                        buffer = ""
                    else:
                        buffer += received_message
                else:
                    pass

    def update_latest_tracker_message(self, buffer):
        try:
            d = json.loads(json.loads(buffer))
            vive_tracker_message = ViveTrackerMessage.parse_obj(d)
            self.latest_tracker_message = vive_tracker_message
            print(self.latest_tracker_message)
        except Exception as e:
            self.logger.error(f"Unable to parse buffer [{buffer}]")
            #
            #     buffer = conn.recv(1)
            #     buffer = buffer.decode()
            #     if buffer == ";":
            #         conn.close()
            #         d = json.loads(json.loads(data))
            #         data = ""
            #         vive_tracker_message = ViveTrackerMessage.parse_obj(d)
            #         self.latest_tracker_message = vive_tracker_message
            #         break
            #     elif buffer:
            #         data += buffer
            #     else:
            #         break
            # print(self.latest_tracker_message)

    def run_threaded(self):
        pass

    def shutdown(self):
        self.socket.close()
