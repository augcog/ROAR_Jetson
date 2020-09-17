import serial

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;


class Sender:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, writeTimeout=1)
        self.prev_throttle = 1500  # record previous throttle, set to neutral initially 
        self.prev_steering = 1500  # record previous steering, set to neutral initially

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        if throttle >= 0:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_MAX - MOTOR_NEUTRAL) * throttle)
        else:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_NEUTRAL - MOTOR_MIN) * throttle)
        steering_send = int(THETA_MIN + (steering / 2 + 0.5) * (THETA_MAX - THETA_MIN))
        # only send new msg when throttle or steering changes
        if self.prev_throttle != throttle_send or self.prev_steering != steering_send:
            self.ser.write('& {} {}\n'.format(throttle_send, steering_send).encode('ascii'))
            self.prev_throttle = throttle_send
            self.prev_steering = steering_send
            print(f'sender writes throttle {throttle_send} steering {steering_send}')

    def shutdown(self):
        print('sender shutdown...')
        self.ser.write('& 1500 1500\n'.encode('ascii'))
        print(f'...sender writes throttle 1500 steering 1500')
