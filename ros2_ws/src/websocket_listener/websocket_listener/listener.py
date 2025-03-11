import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Charger les données du fichier JSON
def load_data():
    try:
        with open("data.json", "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"x": 0.0, "y": 0.0, "z": 0.0}  # Valeur par défaut

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer_period = 0.5  # secondes
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        data = load_data()  # Charger les données depuis le fichier
        msg = String()
        msg.data = f'X: {data["x"]:.2f}, Y: {data["y"]:.2f}, Z: {data["z"]:.2f}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass  # Arrêter proprement avec Ctrl+C

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
