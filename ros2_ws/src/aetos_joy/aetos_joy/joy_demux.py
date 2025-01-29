import rclpy
from rclpy.node import Node
import pygame
import os
import time
from aetos_msgs.msg import Velocity 

# This node takes input from a Xbox controller and publishes it as a Velocity vector for x, y, and z on 

os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

class JoyDemux(Node):
    def __init__(self):
        super().__init__('joy_demux')

        # Initialize publisher
        self.velocity_publisher = self.create_publisher(Velocity, 'velocity_topic', 10)

        # Check for controller
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller connected!")
            raise RuntimeError("No Xbox controller found. Please connect one and try again.")

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Connected to Xbox Controller: {self.controller.get_name()}")

        # Create timer to read joystick input, reading at 10Hz
        self.timer = self.create_timer(0.1, self.process_joystick_input)

        # Value storage for comparison
        self.prev_velocity = None
        self.last_print_time = time.time()
        
    def process_joystick_input(self):
        """Reads joystick values, prints them, and publishes them as a Velocity message."""
        pygame.event.pump()

    
        dpad_x = self.controller.get_hat(0)[0]  
        dpad_y = self.controller.get_hat(0)[1]  
        
    
        button_a = self.controller.get_button(0) 
        button_x = self.controller.get_button(3)  
        
        velocity_x = float(dpad_x)
        velocity_y = float(dpad_y)
        velocity_z = float(button_x - button_a)  # Y positif, A negatif

       
        velocity_msg = Velocity()
        velocity_msg.data = [velocity_x, velocity_y, velocity_z]
        self.velocity_publisher.publish(velocity_msg)

       
        current_time = time.time()
        if self.prev_velocity != (velocity_x, velocity_y, velocity_z) or current_time - self.last_print_time > 1.0:
            print("\033c", end="")
            print(f"Velocity (X, Y, Z): ({velocity_x:.2f}, {velocity_y:.2f}, {velocity_z:.2f})")
            self.prev_velocity = (velocity_x, velocity_y, velocity_z)
            self.last_print_time = current_time

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
        pygame.quit()  # Properly shut down pygame

if __name__ == '__main__':
    main()
