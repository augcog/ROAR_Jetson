import serial

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;


class Controller:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.new_steering = 0.0
        self.new_throttle = 0.0

    def update(self):
        # update throttle and steering
        pass

    def run_threaded(self, **args):
        if self.new_throttle >= 0:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_MAX - MOTOR_NEUTRAL) * self.new_throttle + 0.5)
        else:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_NEUTRAL - MOTOR_MIN) * self.new_throttle + 0.5)
        steering_send = int(THETA_MIN + (self.new_steering / 2 + 0.5) * (THETA_MAX - THETA_MIN) + 0.5)
        self.ser.write('{} {}\n'.format(throttle_send, steering_send).encode())

class NaiveController(Controller):
    def __init__(self):
        Controller.__init__(self)

    def update(self):
        while True:
            try:
                print('input: "throttle steering"')
                x, y = input().split()
                self.new_throttle, self.new_steering = float(x), float(y)
            except:
                pass
