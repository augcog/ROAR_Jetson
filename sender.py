import serial

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;


class Sender:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, writeTimeout=1)
        self.prev_throttle = 0  # record previous throttle 
        self.prev_steering = 0  # record previous steering 

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        if throttle >= 0:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_MAX - MOTOR_NEUTRAL) * throttle)
        else:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_NEUTRAL - MOTOR_MIN) * throttle)
        steering_send = int(THETA_MIN + (steering / 2 + 0.5) * (THETA_MAX - THETA_MIN))
        if self.prev_throttle != throttle_send or self.prev_steering != steering_send:
            self.ser.write('& {} {}\n'.format(throttle_send, steering_send).encode('ascii'))
            self.prev_throttle = throttle_send
            self.prev_steering = steering_send
            print(f'sender write throttle {throttle_send} steering {steering_send}')
