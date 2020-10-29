import socket
import sys
import logging






class ViveTrackerClient:
    def __init__(self, host, port, tracker_name, interval=0.5, buffer_length=1024):
        self.host = host
        self.port = port
        self.tracker_name = tracker_name
        self.interval = interval
        self.buffer_length = buffer_length
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(2)
        self.logger = logging.getLogger(f"Vive Tracker Client for {self.tracker_name}")
        self.logger.info("Tracker Initialized")

    def update(self):
        self.logger.info(f"Start Subscribing to [{self.host}:{self.port}] "
                         f"for [{self.tracker_name}] Vive Tracker Updates")
        buffer: str = ""
        while True:
            try:
                self.socket.sendto(bytes(self.tracker_name + "\n", "utf-8"), (self.host, self.port))
                received = str(self.socket.recv(1024), "utf-8")
                print(received)
            except socket.timeout():
                self.logger.error("Timed out")



    def run_threaded(self):
        pass

    def shutdown(self):
        self.socket.close()




if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s '
                               '- %(levelname)s - %(message)s',
                        level=logging.DEBUG)
    HOST, PORT = "192.168.1.5", 8000
    client = ViveTrackerClient(host=HOST, port=PORT, tracker_name="tracker_1")
    client.update()