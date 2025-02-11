import rclpy
from rclpy.node import Node
from aetos_msgs.msg import EffectorPosition, MotorVelocity
import numpy as np

class EffectorPositionPublisher(Node):
    def __init__(self):
        super().__init__('effector_position_publisher')
        self.position_publisher = self.create_publisher(EffectorPosition, 'effector_position', 10)
        self.velocity_publisher = self.create_publisher(MotorVelocity, 'motor_velocity', 10)
        self.timer = self.create_timer(0.1, self.publish_data)  
        self.t = 0.0  

    def publish_data(self):
        self.t += 0.1  
        msg_pos = EffectorPosition()
        msg_vel = MotorVelocity()
        
        radius = 2.0
        omega = 1.0

        msg_pos.position_x = radius * np.cos(omega * self.t)
        msg_pos.position_y = radius * np.sin(omega * self.t)
        msg_pos.position_z = 5.0  
        
        
        msg_vel.w1 = omega * radius * np.cos(omega * self.t)
        msg_vel.w2 = omega * radius * np.sin(omega * self.t)
        msg_vel.w3 = -msg_vel.w1
        msg_vel.w4 = -msg_vel.w2

        
        self.position_publisher.publish(msg_pos)
        self.velocity_publisher.publish(msg_vel)

        self.get_logger().info(f'Publishing Position: x={msg_pos.position_x:.2f}, y={msg_pos.position_y:.2f}, z={msg_pos.position_z:.2f}')
        self.get_logger().info(f'Publishing Velocities: w1={msg_vel.w1:.2f}, w2={msg_vel.w2:.2f}, w3={msg_vel.w3:.2f}, w4={msg_vel.w4:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = EffectorPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
