import pygame
from pygame.locals import *
import logging
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import VehicleControl


class JetsonKeyboardControl(object):
    def __init__(self, throttle_increment=0.01, steering_increment=0.01):
        self.logger = logging.getLogger(__name__)
        self._steering_increment = steering_increment
        self._throttle_increment = throttle_increment
        self.steering = 0.0
        self.throttle = 0.0
        self.logger.debug("Keyboard Control Initiated")

    def parse_events(self, clock: pygame.time.Clock):
        """
        parse a keystoke event
        Args:
            clock: pygame clock

        Returns:

        """
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                return False, VehicleControl()
        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        return True, VehicleControl(throttle=self.throttle, steering=self.steering)

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self.throttle = min(self.throttle + self._throttle_increment, 1)

        elif keys[K_DOWN] or keys[K_s]:
            self.throttle = max(self.throttle - self._throttle_increment, -1)

        if keys[K_LEFT] or keys[K_a]:
            self.steering = max(self.steering - self._steering_increment, -1)

        elif keys[K_RIGHT] or keys[K_d]:
            self.steering = min(self.steering + self._steering_increment, 1)

        self.throttle, self.steering = round(self.throttle, 5), round(self.steering, 5)

