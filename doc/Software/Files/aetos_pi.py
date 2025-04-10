import asyncio
import websockets
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2 import Preview
import struct
import time

# =======================
# Configuration
# =======================
SERVER_IP = "0.0.0.0"   # IP du serveur
PORT = 8765                 # Port WebSocket

# Initialiser la caméra avec picamera2
picam2 = Picamera2()
picam2.start_preview(Preview.NULL)  # Désactiver la prévisualisation
picam2.start()

# Paramètres de l'objet suivi
real_width = 15           # Largeur réelle de l'objet (cm)
focal_length = 65         # Distance focale (pixels)
min_target_distance = 13  # Distance cible minimale (cm)
max_target_distance = 16  # Distance cible maximale (cm)

# Dead zone (20 % du centre de l'image)
dead_zone_size = 0.2

# =======================
# Fonction d'envoi WebSocket
# =======================
async def send_data():
    while True:
        try:
            print(f"Tentative de connexion au serveur WebSocket à {SERVER_IP}:{PORT}...")
            async with websockets.connect(f"ws://{SERVER_IP}:{PORT}") as websocket:
                print("Connecté au serveur WebSocket.")
                
                while True:
                    # Capture de l'image
                    image = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGR2RGB)
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                    # Détection de la couleur rose
                    lower_pink = np.array([0, 100, 100])
                    upper_pink = np.array([10, 255, 255])
                    mask = cv2.inRange(hsv, lower_pink, upper_pink)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    speed_x = 0.0
                    speed_y = 0.0
                    speed_z = 0.0

                    if contours:
                        # Trouver le plus grand objet détecté
                        largest_contour = max(contours, key=cv2.contourArea)
                        x, y, w, h = cv2.boundingRect(largest_contour)

                        # Dessiner un carré autour de l'objet détecté
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)  # Carré vert

                        if w > 0:
                            # Calcul de la distance
                            distance = (focal_length * real_width) / w
                            center_x = x + w // 2
                            center_y = y + h // 2

                            # Normalisation
                            center_x_normalized = (center_x - image.shape[1] // 2) / (image.shape[1] // 2)
                            center_y_normalized = (center_y - image.shape[0] // 2) / (image.shape[0] // 2)

                            # Gestion de la dead zone
                            if abs(center_x_normalized) < dead_zone_size and abs(center_y_normalized) < dead_zone_size:
                                speed_x = 0.0
                                speed_y = 0.0
                                print("Objet dans la dead zone : arrêt des mouvements X et Y")
                            else:
                                # Calcul des vitesses
                                speed_x = center_x_normalized
                                speed_y = -center_y_normalized
                                
                            if distance < min_target_distance:
                                speed_z = - (min_target_distance - distance) / min_target_distance
                            elif distance > max_target_distance:
                                speed_z = (distance - max_target_distance) / max_target_distance
                            else:
                                speed_z = 0.0
                                                        

                    # Encodage de l'image en JPEG
                    _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    img_bytes = buffer.tobytes()

                    # Fusion des données (position + image)
                    position_data = struct.pack('fff', speed_x, speed_y, speed_z)
                    message = position_data + img_bytes

                    # Envoi des données binaires
                    await websocket.send(message)
                    
                    print(f"Envoi: x={speed_x:.2f}, y={speed_y:.2f}, z={speed_z:.2f}")
                    
                    await asyncio.sleep(0.05)  # Fréquence d'envoi de 10 Hz

        except Exception as e:
            print(f"Erreur de connexion WebSocket: {e}")
            print("Nouvelle tentative dans 3 secondes...")
            await asyncio.sleep(3)  # Attendre 3 secondes avant de réessayer

# =======================
# Lancement du client
# =======================
asyncio.run(send_data())
