# MAC ADRESS XBOX CONTROLLER : 44:16:22:20:FB:5E
# Max raw values from controller : [-32767 , 32767]  down and right is positive
import rclpy
from rclpy.node import Node
import pygame
import os
import time

# *** Joystick input info *** 

#Left : Controls X and Y velocities. 
#       +X right direction
#       +Y down direction

#Right : Controls Z velocities
#       +Z down direction

os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

class JoyDemux(Node):
    def __init__(self):
        super().__init__('joy_demux')

        # Check for controller
        pygame.init()
        pygame.joystick.init()

        
        if pygame.joystick.get_count() == 0:
            print("No Xbox controller connected!")  
            raise RuntimeError("No Xbox controller found. Please connect one and try again.")

        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Connected to Xbox Controller: {self.controller.get_name()}")

        
        self.timer = self.create_timer(0.1, self.print_raw_inputs)

        # Value storage for comparison
        self.prev_left_joystick = None
        self.prev_right_joystick = None
        self.last_print_time = time.time()

    def print_raw_inputs(self):
        """Reads and prints the raw joystick values from the controller."""
        pygame.event.pump()  

        # Raw joystick values 
        left_joystick = [
            self.controller.get_axis(0),  # Left stick X-axis
            self.controller.get_axis(1)   # Left stick Y-axis
        ]
        
        right_joystick = [
            self.controller.get_axis(3)   # Right stick Y-axis
        ]
        
        velocity_x = left_joystick[0]
        velocity_y = left_joystick[1]
        velocity_z = right_joystick[0]
        

        
        current_time = time.time()
        if (left_joystick != self.prev_left_joystick or
            right_joystick != self.prev_right_joystick or
            current_time - self.last_print_time > 1.0):
            
             
            print("\033c", end="")
            
            print(f"Velocity (X, Y, Z): ({velocity_x:.2f}, {velocity_y:.2f}, {velocity_z:.2f})") 

            # print(f"Left Joystick (X, Y): {left_joystick}")
            # print(f"Right Joystick (X, Y): {right_joystick}")

            # Update the previous joystick values and last print time
            self.prev_left_joystick = left_joystick
            self.prev_right_joystick = right_joystick
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
