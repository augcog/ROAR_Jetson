from ROAR.roar_autonomous_system.utilities_module.data_structures_models import SensorsData
from ROAR.roar_autonomous_system.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.ROAR_Jetson.jetson_vehicle import Vehicle as JetsonVehicle
from typing import Optional, Tuple
from ROAR.ROAR_Jetson.jetson_cmd_sender import JetsonCommandSender
from ROAR.roar_autonomous_system.agent_module.agent import Agent
from ROAR.bridges.jetson_bridge import JetsonBridge
from ROAR.ROAR_Jetson.camera import RS_D435i
import logging
import pygame
from ROAR.ROAR_Jetson.jetson_keyboard_control import JetsonKeyboardControl
import numpy as np
from ROAR.ROAR_Jetson.jetson_config import JetsonConfig


class JetsonRunner:
    """
    In charge of maintaining the state of the game.
    Drive command for agent to move next step
    Drive command for jetson to issue next command
    Update PyGame visualizations and controls parsing
    """
    def __init__(self, agent: Agent, jetson_config: JetsonConfig):
        self.jetson_vehicle: JetsonVehicle = JetsonVehicle()
        self.jetson_config = jetson_config
        self.agent = agent
        self.jetson_bridge = JetsonBridge()
        self.logger = logging.getLogger("Jetson Runner")
        self.display: Optional[pygame.display] = None
        self.controller = JetsonKeyboardControl()
        if jetson_config.initiate_pygame:
            self.setup_pygame()
        self.setup_jetson_vehicle()
        self.auto_pilot = True
        self.logger.info("Jetson Vehicle Connected and Intialized")

    def setup_pygame(self):
        self.pygame_initiated = False
        self.logger.info("Jetson Vehicle Connected and Intialized")

    def setup_pygame(self):
        """
        Initiate pygame
        Returns:

        """
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((self.jetson_config.pygame_display_width,
                                                self.jetson_config.pygame_display_height),
                                               pygame.OPENGL)
                                               # pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.pygame_initiated = True


    def setup_jetson_vehicle(self):
        """
        Add component to JetsonVehicle instance
        Returns:
            None
        """
        self.jetson_vehicle.add(JetsonCommandSender(), inputs=['throttle', 'steering'], threaded=True)
        self.jetson_vehicle.add(RS_D435i(image_w=self.agent.front_rgb_camera.image_size_x,
                                         image_h=self.agent.front_rgb_camera.image_size_y,
                                         image_output=True), threaded=True)

    def start_game_loop(self, use_manual_control=False):
        self.logger.info("Starting Game Loop")
        try:

            clock = pygame.time.Clock()
            should_continue = True
            while should_continue:
                clock.tick_busy_loop(60)
                # pass throttle and steering into the bridge
                sensors_data, vehicle = self.convert_data()

                # run a step of agent
                vehicle_control = VehicleControl()
                if self.auto_pilot:
                    vehicle_control: VehicleControl = self.agent.run_step(sensors_data=sensors_data, vehicle=vehicle)
                # manual control always take precedence
                if use_manual_control:
                    should_continue, vehicle_control = self.update_pygame(clock=clock)
                # self.logger.debug(f"Vehicle Control = [{vehicle_control}]")
                # pass the output into sender to send it
                self.jetson_vehicle.update_parts(new_throttle=vehicle_control.throttle,
                                                 new_steering=vehicle_control.steering)
        except KeyboardInterrupt:
            self.logger.info("Keyboard Interrupt detected. Safely quitting")
            self.jetson_vehicle.stop()
        except Exception as e:
            self.logger.error(f"Something bad happened: [{e}]")
        finally:
            self.jetson_vehicle.stop()

    def convert_data(self) -> Tuple[SensorsData, Vehicle]:
        """
        Convert sensor and vehicle state data from source to agent
        Returns:
            SensorsData and Vehicle state.
        """
        sensors_data: SensorsData = self.jetson_bridge.convert_sensor_data_from_source_to_agent(
            source={
                "front_rgb": self.jetson_vehicle.front_rgb_img,
                "rear_rgb": None,
                "front_depth": self.jetson_vehicle.front_depth_img,
                "imu": None
            }
        )
        new_vehicle = self.jetson_bridge.convert_vehicle_from_source_to_agent(self.jetson_vehicle)
        return sensors_data, new_vehicle

    def update_pygame(self, clock) -> Tuple[bool, VehicleControl]:
        """
        Update the pygame window, including parsing keypress

        Args:
            clock: pygame clock

        Returns:
            bool - whether to continue the game
            VehicleControl - the new VehicleControl cmd by the keyboard
        """
        should_continue, vehicle_control = self.controller.parse_events(clock=clock)
        if self.display is not None and self.agent.front_rgb_camera.data is not None:
            array: np.ndarray = self.agent.front_rgb_camera.data.copy()[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.display.blit(surface, (0, 0))
        pygame.display.flip()
        return should_continue, vehicle_control
