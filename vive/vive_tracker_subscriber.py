from ROAR_Jetson.vive.models import ViveTrackerMessage
import socket
import json
import logging
from typing import Optional, List


class ViveTrackerSubscriber:
    def __init__(self, host: str, port: int, tracker_name: str, buffer_length=1024):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.latest_tracker_message: Optional[ViveTrackerMessage] = None
        self.logger = logging.getLogger("Vive Tracker Subscriber")
        self.logger.debug("Initialization Success")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.buffer_length = buffer_length

    def update(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        buffer: str = ""
        # self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        # self.socket.listen(5)
        try:
            while True:
                while True:
                    received_message, addr = self.socket.recvfrom(self.buffer_length)
                    received_message = received_message.decode()
                    # received_message = conn.recv(self.buffer_length).decode()
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
        except WindowsError as e:
            self.logger.error(f"{e}"
                              f"This error is known and can be safely ignored. "
                              f"However, if you are bothered, please contact admin to expedite fix patch. ")
        except Exception as e:
            self.logger.error(f"Error: {e}")

    def update_latest_tracker_message(self, buffer):
        try:
            d = json.loads(json.loads(buffer))
            vive_tracker_message = ViveTrackerMessage.parse_obj(d)
            if vive_tracker_message.device_name == self.tracker_name:
                self.latest_tracker_message = vive_tracker_message
            # print(self.latest_tracker_message)
            self.logger.info(self.latest_tracker_message)
        except Exception as e:
            self.logger.error(f"Error: {e} \nMaybe it is related to unable to parse buffer [{buffer}]. ")

    def run_threaded(self):
        pass

    def shutdown(self):
        self.socket.close()
