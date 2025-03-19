import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import rclpy
import threading
from rclpy.node import Node
from aetos_msgs.msg import EffectorPosition, MotorVelocity, EncoderValues
import time

class Sim(Node):
    def __init__(self):
        super().__init__('sim')
        self.subscription = self.create_subscription(EffectorPosition, 'aetos/control/position', self.effector_position_callback, 10)
        self.velocity_subscription = self.create_subscription(MotorVelocity, 'aetos/control/velocity', self.motor_velocity_callback, 10)
        self.encoder_publisher = self.create_publisher(EncoderValues, 'aetos/control/sim_encoder', 10)
        
        self.position = [0.0, 0.0, 0.0] 
        self.last_velocity_time = time.time()
        self.last_motor_velocities = [0.0, 0.0, 0.0, 0.0]
        self.encoder_values = EncoderValues()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.init_plot()
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=50, blit=False)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.ros_thread.start()

        plt.show()

    def init_plot(self):
        size = 1.5
        beam_thickness = 0.01
        beam_height = 0.01
        square_size = 1  
        vertical_height = 1  

        beams = [
            (0, 0, 1, square_size, beam_thickness, beam_height),  
            (0, square_size - beam_thickness, 1, square_size, beam_thickness, beam_height),  
            (0, 0, 1, beam_thickness, square_size, beam_height),  
            (square_size - beam_thickness, 0, 1, beam_thickness, square_size, beam_height),  
        ]

        vertical_beams = [
            (0, 0, 0, beam_thickness, beam_thickness, vertical_height),  
            (0, square_size, 0, beam_thickness, beam_thickness, vertical_height),  
            (square_size, square_size, 0, beam_thickness, beam_thickness, vertical_height),  
            (square_size, 0, 0, beam_thickness, beam_thickness, vertical_height),  
        ]

        self.anchor_points = [
            (0, 0, 0),  
            (1, 0, 0),  
            (0, 1, 0),  
            (1, 1, 0),  
    ]

        for x, y, z, dx, dy, dz in beams:
            self.ax.bar3d(x, y, z, dx, dy, dz, color='black')
        for x, y, z, dx, dy, dz in vertical_beams:
            self.ax.bar3d(x, y, z, dx, dy, dz, color='black')

        self.ax.set_xlim([0, size])
        self.ax.set_ylim([0, size])
        self.ax.set_zlim([0, size])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.effector, = self.ax.plot([0], [0], [5], 'ro', markersize=4)
        self.cables = [self.ax.plot([], [], [], 'gray', linestyle='-', linewidth=1)[0] for _ in range(4)]

    def effector_position_callback(self, msg):
        """Handles incoming effector position messages."""
        self.position = [msg.position_x, msg.position_y, msg.position_z]

    def motor_velocity_callback(self, msg):
        """Handles incoming motor velocity messages and calculates encoder values."""
        current_time = time.time()
        time_diff = current_time - self.last_velocity_time
        
        variable_temp1 = msg.omega1 * time_diff
        varuable_temp2 = msg.omega2 * time_diff
        variable_temp3 = msg.omega3 * time_diff 
        variable_temp4 = msg.omega4 * time_diff
        
        if math.isnan(variable_temp1) | math.isnan(varuable_temp2) | math.isnan(variable_temp3) | math.isnan(variable_temp4):
            self.encoder_values.angle1 = self.encoder_values.angle1
            self.encoder_values.angle2 = self.encoder_values.angle2
            self.encoder_values.angle3 = self.encoder_values.angle3
            self.encoder_values.angle4 = self.encoder_values.angle4
            
        else:
            self.encoder_values.angle1 += msg.omega1 * time_diff
            self.encoder_values.angle2 += msg.omega2 * time_diff
            self.encoder_values.angle3 += msg.omega3 * time_diff
            self.encoder_values.angle4 += msg.omega4 * time_diff
        
        self.encoder_publisher.publish(self.encoder_values)


        self.last_motor_velocities = [msg.omega1, msg.omega2, msg.omega3, msg.omega4]
        
        
        self.last_velocity_time = current_time

    def update(self, frame):
        effector_x, effector_y, effector_z = self.position
        self.effector.set_data([effector_x], [effector_y])
        self.effector.set_3d_properties([effector_z])

        for i, (x, y, z) in enumerate(self.anchor_points):
            self.cables[i].set_data([x, effector_x], [y, effector_y])
            self.cables[i].set_3d_properties([z, effector_z])
        
        return [self.effector] + self.cables

def main(args=None):
    rclpy.init(args=args)
    node = Sim()
    try:
        while rclpy.ok():
            plt.pause(0.1)  # Permet d'interagir avec le graphe sans bloquer
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
