import math


class Mapper:
    def __init__(self, cfg):
        self.cfg = cfg

    def throttle_to_pwm(self, x):
        return int(self._throttle_to_pwm(x) * self.cfg.THROTTLE_MAX * self.cfg.MOTOR_PWM_SCALE + self.cfg.MOTOR_PWM_NEUTRAL + 0.5)

    def throttle_from_pwm(self, x):
        return self._throttle_from_pwm((float(x) - self.cfg.MOTOR_PWM_NEUTRAL) / self.cfg.MOTOR_PWM_SCALE)

    def steering_to_pwm(self, x):
        return int(self._steering_to_pwm(x) * self.cfg.STEERING_MAX * self.cfg.THETA_PWM_SCALE + self.cfg.THETA_PWM_NEUTRAL + 0.5)

    def steering_from_pwm(self, x):
        return self._steering_from_pwm((float(x) - self.cfg.THETA_PWM_NEUTRAL) / self.cfg.THETA_PWM_SCALE)

    # All below map from [-1,1] to [-1,1]
    def _throttle_to_pwm(self, x):
        if abs(x) < 0.02:
            return 0
        if x > 0:
            return 0.937 * (x - 0.02) ** 2 + 0.1
        else:
            - x ** 2;

    def _throttle_from_pwm(self, x):
        sgn = 1 if x > 0 else -1
        x = abs(x)
        if x < 0.1:
            y = x * 0.2
        elif x < 0.85:
            y = 0.64 * x - 0.044
        else:
            y = 3.3 * x - 2.3
        return sgn * y

    def _steering_to_pwm(self, x):
        return x

    def _steering_from_pwm(self, x):
        return x
