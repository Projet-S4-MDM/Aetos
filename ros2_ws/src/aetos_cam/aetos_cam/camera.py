import asyncio
import websockets
import base64
import cv2
import numpy as np
import time
import threading
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
lock = threading.Lock()  # Pour gérer l'accès aux variables partagées

# =======================
# Fonction WebSocket (thread séparé)
# =======================
async def handle_websocket(websocket):
    """ Réception WebSocket avec binaire direct """
    global last_message_time

    async for message in websocket:
        try:
            # Séparer les données de position et l'image
            header_size = 12  # 3 x float (4 octets chacun)
            
            if len(message) >= header_size:
                with lock:
                    # Extraire les coordonnées (3x float)
                    shared_data["x"] = np.frombuffer(message[0:4], dtype=np.float32)[0]
                    shared_data["y"] = np.frombuffer(message[4:8], dtype=np.float32)[0]
                    shared_data["z"] = np.frombuffer(message[8:12], dtype=np.float32)[0]

                    # Extraire l'image (binaire restant)
                    img_data = message[header_size:]
                    if img_data:
                        np_array = np.frombuffer(img_data, np.uint8)
                        image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
                        if image is not None:
                            shared_data["image"] = image

                    last_message_time = time.time()

        except Exception as e:
            print(f"[WebSocket] Erreur: {e}")

async def start_websocket_server():
    """ Démarrer le serveur WebSocket """
    server = await websockets.serve(handle_websocket, "0.0.0.0", 8765, max_size=2**23)
    print("[WebSocket] Serveur démarré sur le port 8765")
    await server.wait_closed()

# =======================
# Node ROS2 pour publier les données
# =======================
class DataPublisherNode(Node):
    """ Nœud ROS2 pour publier les données Vector3 """
    def __init__(self):
        super().__init__("data_publisher")
        self.publisher_ = self.create_publisher(Vector3, "data_topic", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        """ Callback ROS2 pour publier les données """
        global last_message_time

        now = time.time()

        with lock:
            if now - last_message_time > 1.0:
                # Timeout : réinitialisation des valeurs
                print("[Sécurité] Timeout dépassé. Réinitialisation des données.")
                shared_data["x"] = 0.0
                shared_data["y"] = 0.0
                shared_data["z"] = 0.0

            # Publier les coordonnées (forcer la conversion en float natif)
            msg = Vector3()
            msg.x = float(shared_data["x"])  
            msg.y = float(shared_data["y"])  
            msg.z = float(shared_data["z"])  

            self.publisher_.publish(msg)
            self.get_logger().info(f"[ROS2] Publication: x={msg.x}, y={msg.y}, z={msg.z}")


# =======================
# Thread pour afficher les images OpenCV
# =======================
def image_display_thread():
    """ Affichage des images dans un thread séparé """
    while rclpy.ok():
        with lock:
            if shared_data["image"] is not None:
                cv2.imshow("Image Reçue", shared_data["image"])
                cv2.waitKey(1)
        time.sleep(0.05)

# =======================
# Fonction principale
# =======================
def main():
    rclpy.init()
    node = DataPublisherNode()

    # Lancer le serveur WebSocket dans un thread séparé
    websocket_thread = threading.Thread(target=asyncio.run, args=(start_websocket_server(),))
    websocket_thread.daemon = True
    websocket_thread.start()

    # Lancer le thread d'affichage d'images
    img_thread = threading.Thread(target=image_display_thread)
    img_thread.daemon = True
    img_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("[MAIN] Arrêté par utilisateur.")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
