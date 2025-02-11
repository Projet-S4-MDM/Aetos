import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import rclpy
from rclpy.node import Node

class CableDrivenRobot(Node):
    def __init__(self):
        super().__init__('cable_robot_node')
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.init_plot()
        self.ani = animation.FuncAnimation(self.fig, self.update, frames=100, interval=50, blit=False)
        plt.show()

    def init_plot(self):
        size = 10  
        beam_thickness = 0.2
        beam_height = 0.2
        square_size = 10  
        vertical_height = 10  

        beams = [
            (-square_size / 2, -square_size / 2, 0, square_size, beam_thickness, beam_height),  
            (-square_size / 2, square_size / 2 - beam_thickness, 0, square_size, beam_thickness, beam_height),  
            (-square_size / 2, -square_size / 2, 0, beam_thickness, square_size, beam_height),  
            (square_size / 2 - beam_thickness, -square_size / 2, 0, beam_thickness, square_size, beam_height),  
        ]

        vertical_beams = [
            (-square_size / 2, -square_size / 2, 0, beam_thickness, beam_thickness, vertical_height),  
            (-square_size / 2, square_size / 2 - beam_thickness, 0, beam_thickness, beam_thickness, vertical_height),  
            (square_size / 2 - beam_thickness, -square_size / 2, 0, beam_thickness, beam_thickness, vertical_height),  
            (square_size / 2 - beam_thickness, square_size / 2 - beam_thickness, 0, beam_thickness, beam_thickness, vertical_height),  
        ]

        self.anchor_points = [
            (-square_size / 2, -square_size / 2, vertical_height),  
            (-square_size / 2, square_size / 2 - beam_thickness, vertical_height),  
            (square_size / 2 - beam_thickness, -square_size / 2, vertical_height),  
            (square_size / 2 - beam_thickness, square_size / 2 - beam_thickness, vertical_height),  
        ]

        for x, y, z, dx, dy, dz in beams:
            self.ax.bar3d(x, y, z, dx, dy, dz, color='black')
        for x, y, z, dx, dy, dz in vertical_beams:
            self.ax.bar3d(x, y, z, dx, dy, dz, color='black')

        self.ax.set_xlim([-size, size])
        self.ax.set_ylim([-size, size])
        self.ax.set_zlim([0, size])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.effector, = self.ax.plot([0], [0], [5], 'ro', markersize=8)
        self.cables = [self.ax.plot([], [], [], 'gray', linestyle='-', linewidth=1)[0] for _ in range(4)]

    def update(self, frame):
        theta = 2 * np.pi * frame / 100
        effector_x = 2 * np.cos(theta)
        effector_y = 2 * np.sin(theta)
        effector_z = 5
        self.effector.set_data([effector_x], [effector_y])
        self.effector.set_3d_properties([effector_z])

        for i, (x, y, z) in enumerate(self.anchor_points):
            self.cables[i].set_data([x, effector_x], [y, effector_y])
            self.cables[i].set_3d_properties([z, effector_z])
        
        return [self.effector] + self.cables


def main(args=None):
    rclpy.init(args=args)
    node = CableDrivenRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
