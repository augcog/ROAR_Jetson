import socket
import serial
import struct
from utils import Mapper

COMMAND_THROTTLE = 0
COMMAND_STEERING = 1
UDP_PORT = 7788


class Receiver:
    def __init__(self, client_ip, cfg):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.old_steering = 0.0
        self.old_throttle = 0.0
        self.new_steering = 0.0
        self.new_throttle = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_ip = client_ip
        self.mapper = Mapper(cfg)

    def update(self):
        while True:
            vel_wheel = self.ser.readline()
            vel_wheel = str(vel_wheel)
            vel_wheel = vel_wheel[2:][:-5]
            vel_wheel = vel_wheel.split()
            try:
                throttle, steering = vel_wheel
                throttle = float(throttle)
                steering = float(steering)
            except:
                continue
            self.new_throttle = self.mapper.throttle_from_pwm(throttle)
            self.new_steering = self.mapper.steering_from_pwm(steering)

    def run_threaded(self, **args):
        if (self.new_throttle != self.old_throttle):
            msg = struct.pack('>Ii', COMMAND_THROTTLE, int(self.new_throttle * 32767))
            self.sock.sendto(msg, (self.client_ip, UDP_PORT))
            self.old_throttle = self.new_throttle
        if (self.new_steering != self.old_steering):
            msg = struct.pack('>Ii', COMMAND_STEERING, int(self.new_steering * 32767))
            self.sock.sendto(msg, (self.client_ip, UDP_PORT))
            self.old_steering = self.new_steering
