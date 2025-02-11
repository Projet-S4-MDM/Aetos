import rclpy
from rclpy.node import Node
from aetos_msgs.msg import EncoderValues

class EncoderValuesSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_values_subscriber')
        self.subscription = self.create_subscription(
            EncoderValues, 
            'encoder_values', 
            self.encoder_values_callback, 
            10
        )
        self.subscription  # prevent unused variable warning

    def encoder_values_callback(self, msg):
        """Handles incoming encoder values and prints them."""
        self.get_logger().info(f"Received Encoder Values: angle1={msg.angle1:.2f}, angle2={msg.angle2:.2f}, angle3={msg.angle3:.2f}, angle4={msg.angle4:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderValuesSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
