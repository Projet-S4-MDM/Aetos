import rclpy
from rclpy.node import Node
import pygame
import os
import time
import sys
from aetos_msgs.msg import Velocity 

class JoyDemux(Node):
    def __init__(self):
        super().__init__('joy_demux')
        
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

        self.velocity_publisher = self.create_publisher(Velocity, 'aetos/velocity/teleop', 10)

        pygame.init()
        pygame.joystick.init()

        self.controller = None
        self.controller_check_timer = self.create_timer(1.0, self.check_controller)  # Check every 1 second
        self.joystick_timer = None  # Will be created once a controller is detected
        
    def check_controller(self):
        """Checks for a connected controller and initializes it if found."""
        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            self.get_logger().info(f"Connected to Xbox Controller: {self.controller.get_name()}")

            # Start joystick input processing
            if self.joystick_timer is None:
                self.joystick_timer = self.create_timer(0.01, self.process_joystick_input)

        else:
            self.get_logger().warn("No Xbox controller detected. Retrying...")

    def process_joystick_input(self):
        """Reads joystick values and publishes them as a Velocity message."""
        pygame.event.pump()

        dpad_x = self.controller.get_hat(0)[0]  
        dpad_y = self.controller.get_hat(0)[1]  
    
        button_a = self.controller.get_button(0) 
        button_x = self.controller.get_button(3)  
        
        velocity_x = float(dpad_x)
        velocity_y = float(dpad_y)
        velocity_z = float(button_x - button_a) 

        velocity_msg = Velocity()
        velocity_msg.velocity_x = velocity_x
        velocity_msg.velocity_y = velocity_y
        velocity_msg.velocity_z = velocity_z
        self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyDemux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()  

if __name__ == '__main__':
    main()
