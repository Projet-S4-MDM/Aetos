import rclpy
import math
from rclpy.node import Node
from aetos_msgs.msg import EffectorPosition

class EffectorPositionPublisher(Node):
    def __init__(self):
        super().__init__('effector_position_publisher')
        self.publisher_ = self.create_publisher(EffectorPosition, 'effector_position', 10)
        self.timer = self.create_timer(0.1, self.publish_position) 
        self.t = 0.0  
        self.radius = 1.0  
        self.z_amplitude = 0.5  
        self.speed = 0.1  

    def publish_position(self):
        msg = EffectorPosition()
        self.x = self.radius * math.cos(self.t)
        self.y = self.radius * math.sin(self.t)
        self.z = self.z_amplitude * math.sin(2 * self.t)  
        msg.position_x, msg.position_y, msg.position_z = self.x, self.y, self.z
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.position_x:.2f}, y={msg.position_y:.2f}, z={msg.position_z:.2f}')
        
        self.t += self.speed  


def main(args=None):
    rclpy.init(args=args)
    node = EffectorPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
