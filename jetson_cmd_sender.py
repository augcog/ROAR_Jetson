import serial
import logging
import sys
import time
import numpy as np
from typing import List, Tuple

MOTOR_MAX = 1750
MOTOR_MIN = 800
MOTOR_NEUTRAL = 1500
THETA_MAX = 3000
THETA_MIN = 0


class JetsonCommandSender:
    def __init__(self, min_command_time_gap: float = 0.1):
        if 'win' in sys.platform:
            self.ser = serial.Serial('COM4', 115200, timeout=1, writeTimeout=1)
        else:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, writeTimeout=1)

        self.prev_throttle = 1500  # record previous throttle, set to neutral initially 
        self.prev_steering = 1500  # record previous steering, set to neutral initially
        self.last_cmd_time = None
        # time in seconds between two commands to avoid killing the arduino
        self.min_command_time_gap = min_command_time_gap
        self.agent_throttle_range: List = [-1, 1]
        self.agent_steering_range: List = [-1, 1]
        self.servo_throttle_range: List = [1300, 1700]
        self.servo_steering_range: List = [1300, 1700]

        self.logger = logging.getLogger("Jetson CMD Sender")

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        if self.last_cmd_time is None:
            self.last_cmd_time = time.time()
        elif time.time() - self.last_cmd_time > self.min_command_time_gap:
            self.send_cmd(throttle=throttle, steering=steering)
            self.last_cmd_time = time.time()

    def send_cmd(self, throttle, steering):
        throttle_send, steering_send = self.map_control(throttle, steering)
        try:
            self.send_cmd_helper(new_throttle=throttle_send, new_steering=steering_send)
        except KeyboardInterrupt as e:
            self.logger.debug("Interrupted Using Keyboard")
            exit(0)
        except Exception as e:
            self.logger.error(f"Something bad happened {e}")

    def send_cmd_helper(self, new_throttle, new_steering):
        if self.prev_throttle != new_throttle or self.prev_steering != new_steering:
            serial_msg = '& {} {}\r'.format(new_throttle, new_steering)
            self.logger.debug(f"Sending [{serial_msg.rstrip()}]")
            self.ser.write(serial_msg.encode('ascii'))
            self.prev_throttle = new_throttle
            self.prev_steering = new_steering

    def shutdown(self):
        self.logger.debug('Shutting down')
        for i in range(5):
            self.logger.debug("Sending Neutral Command for safe shutdown")
            self.send_cmd_helper(new_throttle=1500, new_steering=1500)

    def map_control(self, throttle, steering) -> Tuple[int, int]:
        return (int(np.interp(x=throttle,
                              xp=self.agent_throttle_range,
                              fp=self.servo_throttle_range)),
                int(np.interp(x=steering,
                              xp=self.agent_steering_range,
                              fp=self.servo_steering_range)))
