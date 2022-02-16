from sympy import EX, im
import rclpy 
from rclpy.node import MsgType, Node 
from sensor_msgs.msg import Image
import pygame 
import numpy as np 
from typing import Optional, Tuple
from cv_bridge import CvBridge
import cv2 
from pygame import *
import sys 
from . import config as cfg
import logging
from carla_msgs.msg import CarlaEgoVehicleControl


class ManualDriveWithPyGame(Node):
    def __init__(self):
        super().__init__('manual_drive_with_pygame')
        self.pygame_display_width = cfg.config["pygame_display_width"]
        self.pygame_display_height= cfg.config["pygame_display_height"] 
        self.subscription = self.create_subscription(
            msg_type=Image,
            topic="/rgb_streamer/rgb_image",
            callback=self.on_rgb_image_received,
            qos_profile=10
        )
        self.carla_msg_publisher = self.create_publisher(
            msg_type=CarlaEgoVehicleControl,
            topic="/carla/ego_vehicle/vehicle_control_cmd_manual",
            qos_profile=1
        )
        self.rgb: Optional[np.ndarray] = None
        self.bridge = CvBridge()
        self.display = None
        self.clock = pygame.time.Clock()
        self.setup_pygame()
        self.timer = self.create_timer(0.05, self.pygame_keyboard_callback)
        self.manual_controller = ManualControl()


    def pygame_keyboard_callback(self):
        should_continue, control, _ = self.manual_controller.parse_events(clock=self.clock)
        if should_continue is False:
            self.timer.cancel()
            pygame.display.quit()
            pygame.quit()
            rclpy.try_shutdown()
            
        else:
            control = CarlaEgoVehicleControl(
                throttle=control[0],
                steer=control[1]
            )
            self.carla_msg_publisher.publish(control)


    def setup_pygame(self):
        """
        Initiate pygame
        Returns:
        """
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((self.pygame_display_width,
                                                self.pygame_display_height))

    def on_rgb_image_received(self, msg:Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            if self.display is not None and frame is not None:
                # s = frame.shape
                # height = 3 * s[1] // 4
                # min_y = s[0] - height - self.vertical_view_offset
                # max_y = s[0] - self.vertical_view_offset
                # frame = frame[min_y:max_y, :]
                frame = cv2.resize(frame, dsize=(self.pygame_display_width, self.pygame_display_height))
                frame: np.ndarray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).swapaxes(0, 1)
                pygame.surfarray.blit_array(self.display, frame)
                pygame.display.flip()
        except Exception as e:
            print(e)


class ManualControl:
    def __init__(self, throttle_increment=0.05, steering_increment=0.05):
        self.logger = logging.getLogger(__name__)
        self._steering_increment = steering_increment
        self._throttle_increment = throttle_increment
        self.max_reverse_throttle = cfg.config["max_reverse_throttle"]
        self.max_forward_throttle = cfg.config["max_forward_throttle"]
        self.max_steering = cfg.config["max_steering"]

        self.steering_offset = cfg.config["steering_offset"]

        self.gear_throttle_step = 0.05
        self.gear_steering_step = 0.01

        self.vertical_view_offset = 200

        self.left_trigger = 0
        self.right_trigger = 0
        self.use_joystick = False
        try:
            pygame.joystick.init()
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.logger.info(f"Joystick [{self.joystick.get_name()}] detected, Using Joytick")
            self.use_joystick = True
        except Exception as e:
            self.logger.info("No joystick detected. Plz use your keyboard instead")

        self.steering = 0.0
        self.throttle = 0.0

        self.last_switch_press_time = time.get_ticks()
        self.logger.debug("Keyboard Control Initiated")

    def parse_events(self, clock: pygame.time.Clock):
        """
        parse a keystoke event
        Args:
            clock: pygame clock
        Returns:
            Tuple bool, and vehicle control
            boolean states whether quit is pressed. VehicleControl by default has throttle = 0, steering =
        """
        events = pygame.event.get()
        key_pressed = pygame.key.get_pressed()
        for event in events:
            if event.type == pygame.QUIT or key_pressed[K_q] or key_pressed[K_ESCAPE]:
                return False, (0, 0), False
            if event.type == pygame.JOYHATMOTION:
                hori, vert = self.joystick.get_hat(0)
                if vert > 0:
                    self.max_forward_throttle = np.clip(self.max_forward_throttle + self.gear_throttle_step, 0, 1)
                    self.ios_config.max_forward_throttle = self.max_forward_throttle
                elif vert < 0:
                    self.max_forward_throttle = np.clip(self.max_forward_throttle - self.gear_throttle_step, 0, 1)
                    self.ios_config.max_forward_throttle = self.max_forward_throttle

                if hori > 0:
                    self.steering_offset = np.clip(self.steering_offset + self.gear_steering_step, -1, 1)

                elif hori < 0:
                    self.steering_offset = np.clip(self.steering_offset - self.gear_steering_step, -1, 1)

        is_brake = False
        is_switch_auto_pressed = False
        self.throttle, self.steering = self._parse_vehicle_keys(key_pressed)

        if self.use_joystick:
            self.throttle, self.steering = self._parse_joystick()
        else:
            self.throttle, self.steering = self._parse_vehicle_keys(key_pressed)
            if key_pressed[K_UP]:
                self.vertical_view_offset = min(500, self.vertical_view_offset + 5)
            elif key_pressed[K_DOWN]:
                self.vertical_view_offset = max(0, self.vertical_view_offset - 5)
            if key_pressed[K_SPACE]:
                is_brake = True
        if key_pressed[K_m] and time.get_ticks() - self.last_switch_press_time > 100:
            is_switch_auto_pressed = True
            self.last_switch_press_time = time.get_ticks()
        
        t = np.clip(self.throttle, self.max_reverse_throttle, self.max_forward_throttle)
        s = np.clip(self.steering+self.steering_offset, -self.max_steering, self.max_steering)

        return True, (t,s), is_switch_auto_pressed

    def _parse_joystick(self) -> Tuple[float, float]:
        # code to test which axis is your controller using
        # vals = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        if sys.platform == "win32":
            trigger_val: float = self.joystick.get_axis(2)
            right_stick_hori_val = self.joystick.get_axis(4)
            left_stick_vert_val = self.joystick.get_axis(3)
            throttle = -trigger_val
            steering = right_stick_hori_val
            if left_stick_vert_val > 0.5:
                self.vertical_view_offset = min(500, self.vertical_view_offset + 5)
            elif left_stick_vert_val < -0.5:
                self.vertical_view_offset = max(0, self.vertical_view_offset - 5)
            return throttle, steering
        else:
            left_trigger_val: float = self.joystick.get_axis(5)
            right_trigger_val: float = self.joystick.get_axis(4)
            left_joystick_vertical_val = self.joystick.get_axis(1)
            left_joystick_horizontal_val = self.joystick.get_axis(0)
            right_joystick_vertical_val = self.joystick.get_axis(3)
            right_joystick_horizontal_val = self.joystick.get_axis(2)

            # post processing on raw values
            left_trigger_val = (1 + left_trigger_val) / 2
            right_trigger_val = (1 + right_trigger_val) / 2
            throttle = left_trigger_val + (-1 * right_trigger_val)
            steering = right_joystick_horizontal_val
            left_joystick_vertical_val = -1 * left_joystick_vertical_val

            if left_joystick_vertical_val > 0.5:
                self.vertical_view_offset = min(500, self.vertical_view_offset + 5)
            elif left_joystick_vertical_val < -0.5:
                self.vertical_view_offset = max(0, self.vertical_view_offset - 5)
            throttle, steering = 0, 0
            return throttle, steering

    def _parse_vehicle_keys(self, keys) -> Tuple[float, float]:
        """
        Parse a single key press and set the throttle & steering
        Args:
            keys: array of keys pressed. If pressed keys[PRESSED] = 1
        Returns:
            None
        """
        if keys[K_w]:
            self.throttle = min(self.throttle + self._throttle_increment, 1)

        elif keys[K_s]:
            self.throttle = max(self.throttle - self._throttle_increment, -1)
        else:
            self.throttle = 0

        if keys[K_a]:
            self.steering = max(self.steering - self._steering_increment, -1)

        elif keys[K_d]:
            self.steering = min(self.steering + self._steering_increment, 1)
        else:
            self.steering = 0

        if keys[K_LEFT]:
            self.steering_offset = np.clip(self.steering_offset - self.gear_steering_step, -1, 1)
            self.ios_config.steering_offset = self.steering_offset
        elif keys[K_RIGHT]:
            self.steering_offset = np.clip(self.steering_offset + self.gear_steering_step, -1, 1)
            self.ios_config.steering_offset = self.steering_offset
        return round(self.throttle, 5), round(self.steering, 5)

def main(args=None):
    rclpy.init(args=args)
    mdp = ManualDriveWithPyGame()
    try:
        rclpy.spin(mdp)
    except KeyboardInterrupt:
        pass 
    finally:
        mdp.destroy_node()
        # mdp.shutdown()

if __name__ == '__main__':
    main()

