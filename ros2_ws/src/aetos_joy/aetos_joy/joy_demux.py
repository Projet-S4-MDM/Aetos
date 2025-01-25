# MAC ADRESS XBOX CONTROLLER : 44:16:22:20:FB:5E
# Max raw values from controller : [-32767 , 32767]  down and right is positive
import rclpy
from rclpy.node import Node
import pygame
import os

os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"


class JoyRawPrinter(Node):
    def __init__(self):
        super().__init__('joy_raw_printer')

        # Initialize pygame for Xbox controller handling
        pygame.init()
        pygame.joystick.init()

        # Ensure at least one controller is connected
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller connected!")
            raise RuntimeError("No Xbox controller found. Please connect one and try again.")

        # Initialize the first connected controller
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Connected to Xbox Controller: {self.controller.get_name()}")

        # Timer for reading controller input
        self.timer = self.create_timer(0.1, self.print_raw_inputs)

    def print_raw_inputs(self):
        """Reads and prints raw Xbox controller inputs."""
        pygame.event.pump()  # Process controller events

        # Get raw axis values
        axes = [self.controller.get_axis(i) for i in range(self.controller.get_numaxes())]

        # Get raw button values
        buttons = [self.controller.get_button(i) for i in range(self.controller.get_numbuttons())]

        # Get D-pad (hat) values
        hats = [self.controller.get_hat(i) for i in range(self.controller.get_numhats())]

        # Print the raw inputs
        self.get_logger().info(f"Axes: {axes}")
        self.get_logger().info(f"Buttons: {buttons}")
        self.get_logger().info(f"Hats (D-pad): {hats}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyRawPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()  # Properly shut down pygame


if __name__ == '__main__':
    main()
