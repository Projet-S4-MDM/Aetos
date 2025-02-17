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

        self.velocity_publisher = self.create_publisher(Velocity, 'aetos/joy/velocity', 10)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Xbox controller connected!")
            rclpy.shutdown()
            pygame.quit()
            sys.exit(1)
            
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.get_logger().info(f"Connected to Xbox Controller: {self.controller.get_name()}")

        self.timer = self.create_timer(0.1, self.process_joystick_input)

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
        velocity_z = float(button_x - button_a) 

       
        velocity_msg = Velocity()
        velocity_msg.data = [velocity_x, velocity_y, velocity_z]
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
