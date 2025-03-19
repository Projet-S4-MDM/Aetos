import asyncio
import websockets
import json
import base64
import cv2
import numpy as np
import time 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# =======================
# Variables partagées
# =======================
shared_data = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "image": None  # Pour stocker l'image décodée (OpenCV)
}
last_message_time = time.time() 

# =======================
# Fonction pour décoder image base64
# =======================
def decode_image(base64_str):
    try:
        img_data = base64.b64decode(base64_str)
        np_array = np.frombuffer(img_data, np.uint8)
        image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
        return image
    except Exception as e:
        print(f"[Erreur décodage image] {e}")
        return None

# =======================
# Callback WebSocket : reçoit les données string
# =======================
async def handle_websocket(websocket):
    global shared_data, last_message_time
    async for message in websocket:
        print(f"[WebSocket] Message reçu : {message[:50]}...")  # Affiche les 50 premiers caractères

        try:
            # Suppose que c'est du JSON
            received_data = json.loads(message)

            # Mise à jour des variables partagées
            shared_data["x"] = float(received_data.get('x', 0.0))
            shared_data["y"] = float(received_data.get('y', 0.0))
            shared_data["z"] = float(received_data.get('z', 0.0))

            # Si image présente, décoder
            if 'image' in received_data:
                shared_data["image"] = decode_image(received_data['image'])
                print("[WebSocket] Image reçue et décodée.")

            # ✅ Mise à jour de la dernière réception
            last_message_time = time.time()

        except json.JSONDecodeError:
            print("[WebSocket] Erreur: données non valides JSON.")
        except Exception as e:
            print(f"[WebSocket] Erreur: {e}")

# =======================
# Démarrage serveur WebSocket
# =======================
async def start_websocket_server():
    server = await websockets.serve(handle_websocket, "0.0.0.0", 8765, max_size=2**23)
    print("[WebSocket] Serveur démarré sur le port 8765")
    await server.wait_closed()

# =======================
# Node ROS2 pour publier les données
# =======================
class DataPublisherNode(Node):
    def __init__(self):
        super().__init__("data_publisher")
        self.publisher_ = self.create_publisher(Vector3, "data_topic", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        global shared_data, last_message_time
        now = time.time()

        # ✅ Vérifier timeout (1 seconde sans message)
        if now - last_message_time > 1.0:
            print("[Sécurité] Timeout dépassé. Réinitialisation des données.")
            shared_data["x"] = 0.0
            shared_data["y"] = 0.0
            shared_data["z"] = 0.0

        # Publier les données (qu'elles soient à jour ou remises à 0)
        msg = Vector3()
        msg.x = shared_data["x"]
        msg.y = shared_data["y"]
        msg.z = shared_data["z"]
        self.publisher_.publish(msg)
        self.get_logger().info(f"[ROS2] Publication: x={msg.x}, y={msg.y}, z={msg.z}")

        # Si image présente, afficher
        if shared_data["image"] is not None:
            cv2.imshow("Image Reçue", shared_data["image"])
            cv2.waitKey(1)

# =======================
# Fonction principale pour ROS2 + WebSocket
# =======================
def main():
    rclpy.init()
    node = DataPublisherNode()

    loop = asyncio.get_event_loop()
    websocket_task = loop.create_task(start_websocket_server())

    try:
        # Faire tourner ROS2 et WebSocket ensemble
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            loop.run_until_complete(asyncio.sleep(0.01))
    except KeyboardInterrupt:
        print("[MAIN] Arrêté par utilisateur.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()

if __name__ == "__main__":
    main()


