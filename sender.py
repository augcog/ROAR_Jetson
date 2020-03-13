import serial

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;


class Sender:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        if throttle >= 0:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_MAX - MOTOR_NEUTRAL) * throttle + 0.5)
        else:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_NEUTRAL - MOTOR_MIN) * throttle + 0.5)
        steering_send = int(THETA_MIN + (steering / 2 + 0.5) * (THETA_MAX - THETA_MIN) + 0.5)
        self.ser.write('{} {}\n'.format(throttle_send, steering_send).encode())
