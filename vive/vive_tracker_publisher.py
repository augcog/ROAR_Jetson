import socket
import json
from ROAR_Jetson.vive.models import ViveTrackerMessage


class ViveTrackerPublisher:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.host, self.port))

    def send_message(self, data: ViveTrackerMessage):
        data_to_send = self.construct_json_message(data=data)
        self.s.sendall(data_to_send.encode())
        recv = self.s.recv(1024)
        print("Received", repr(recv))

    @staticmethod
    def construct_json_message(data: ViveTrackerMessage) -> str:
        json_data = json.dumps(data.json(), sort_keys=False, indent=2)
        json_data += ";"
        return json_data
