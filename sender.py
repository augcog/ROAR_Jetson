import serial
from utils import Mapper


class Sender:
    def __init__(self, cfg):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.mapper = Mapper(cfg)

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        throttle_send = mapper.throttle_to_pwm(throttle)
        steering_send = mapper.steering_to_pwm(steering)
        self.ser.write('{} {}\n'.format(throttle_send, steering_send).encode())
