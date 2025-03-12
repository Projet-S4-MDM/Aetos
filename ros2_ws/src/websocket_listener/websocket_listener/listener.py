import rclpy
from rclpy.node import Node
from aetos_msgs.msg import Velocity  # Le bon message !
import json

# Charger les données du fichier JSON
def load_data():
    try:
        with open("data.json", "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"x": 0.1, "y": 0.1, "z": 0.1}  # Valeurs par défaut

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        # ✅ On publie sur le bon topic et avec le bon type
        self.publisher_ = self.create_publisher(Velocity, 'aetos/joy/velocity', 10)
        self.timer_period = 0.5  # secondes
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        data = load_data()  # Lire le fichier JSON
        msg = Velocity()   # Créer le message
        # ✅ Remplir correctement les données attendues (tableau de 3 floats)
        msg.data = [float(data["x"]), float(data["y"]), float(data["z"])]
        # Publier le message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Velocity: x={msg.data[0]:.2f}, y={msg.data[1]:.2f}, z={msg.data[2]:.2f}')


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass  # Gestion propre de Ctrl+C

    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
