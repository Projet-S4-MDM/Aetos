import rclpy
from rclpy.node import Node
from aetos_msgs.msg import Velocity

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Velocity,
            'velocity_topic',
            self.velocity_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def velocity_callback(self, msg):
        self.get_logger().info(f"Received Velocity: X={msg.data[0]:.2f}, Y={msg.data[1]:.2f}, Z={msg.data[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
