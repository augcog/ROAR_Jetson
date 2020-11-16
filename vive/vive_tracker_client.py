import socket
import sys
import logging
from typing import Optional
try:
    from ROAR_Jetson.vive.models import ViveTrackerMessage
except:
    from models import ViveTrackerMessage
import json
import time
from typing import Tuple

class ViveTrackerClient:
    def __init__(self, host, port, tracker_name, interval=0.1, buffer_length=512):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.interval = interval
        self.buffer_length = buffer_length
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(3)
        self.latest_tracker_message = None
        self.logger = logging.getLogger(f"Vive Tracker Client [{self.tracker_name}]")
        self.logger.info("Tracker Initialized")

    def update(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        self.socket.bind((self.host, self.port))
        while True:
            try:
                data, addr = self.socket.recvfrom(1024)  # buffer size is 1024 bytes
                # print(data)
                parsed_message, status = self.parse_message(data.decode())
                if status:
                    self.update_latest_tracker_message(parsed_message=parsed_message)

            except socket.timeout:
                self.logger.error("Timed out")
            except ConnectionResetError as e:
                self.logger.error(f"Error: {e}. Retrying")
            except OSError as e:
                pass
            except KeyboardInterrupt:
                exit(1)
            except Exception as e:
                self.logger.debug(e)

    def run_threaded(self):
        pass

    def shutdown(self):
        self.socket.close()

    def update_latest_tracker_message(self, parsed_message):
        try:
            d = json.loads(json.loads(parsed_message))
            vive_tracker_message = ViveTrackerMessage.parse_obj(d)
            if vive_tracker_message.device_name == self.tracker_name:
                self.latest_tracker_message = vive_tracker_message
            # self.logger.debug(self.latest_tracker_message)
        except Exception as e:
            self.logger.error(f"Error: {e} \nMaybe it is related to unable to parse buffer [{parsed_message}]. ")

    def parse_message(self, received_message:str) -> Tuple[str, bool]:
        start = received_message.find("&")
        end = received_message.find("\r")
        if start == -1 or end == -1:
            return "", False
        else:
            return received_message[start+1:end], True


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    HOST, PORT = "192.168.1.19", 8000
    client = ViveTrackerClient(host=HOST, port=PORT, tracker_name="tracker_1")
    client.update()
