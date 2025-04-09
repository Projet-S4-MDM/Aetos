import asyncio
import websockets
import json
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2 import Preview
import base64

# Initialiser la caméra avec picamera2
picam2 = Picamera2()
picam2.start_preview(Preview.NULL)  # Désactiver la prévisualisation
picam2.start()

# Configuration de l'objet suivi
real_width = 10  # Largeur réelle de l'objet (cm)
focal_length = 65  # Distance focale estimée pour la caméra Pi (pixels)

# Plage de distance cible (en cm)
min_target_distance = 13  
max_target_distance = 17  

# Taille de la dead zone (20% du centre de l'image)
dead_zone_size = 0.2  # Ajuste ce pourcentage selon tes besoins

# IP et PORT du serveur WebSocket (l'IP de ton autre ordinateur)
PORT = 8765

# Fonction pour envoyer les données et le flux vidéo au serveur WebSocket
async def send_data():
    while True:  # Boucle infinie pour tenter de se connecter
        try:
            print(f"Tentative de connexion au serveur WebSocket à {SERVER_IP}:{PORT}...")
            async with websockets.connect(f"ws://{SERVER_IP}:{PORT}") as websocket:
                print("Connecté au serveur WebSocket.")
                while True:
                    # Capture de l'image
                    image = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGR2RGB)
                    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
                    # Détection de la couleur rose
                    lower_pink = np.array([30, 50, 50])
                    upper_pink = np.array([90, 255, 255])
                    mask = cv2.inRange(hsv, lower_pink, upper_pink)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    speed_x = 0
                    speed_y = 0
                    speed_z = 0

                    if contours:
                        # Trouver le plus grand objet rose
                        largest_contour = max(contours, key=cv2.contourArea)
                        x, y, w, h = cv2.boundingRect(largest_contour)

                        # Dessiner un carré autour de l'objet détecté
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)  # Carré vert

                        if w > 0:  
                            # Calcul de la distance à l'objet
                            distance = (focal_length * real_width) / w
                            center_x = x + w // 2
                            center_y = y + h // 2

                            # Normalisation des coordonnées
                            center_x_normalized = (center_x - image.shape[1] // 2) / (image.shape[1] // 2)
                            center_y_normalized = (center_y - image.shape[0] // 2) / (image.shape[0] // 2)

                            # Vérification de la dead zone
                            if abs(center_x_normalized) < dead_zone_size and abs(center_y_normalized) < dead_zone_size:
                                speed_x = 0
                                speed_y = 0
                                print("Objet dans la dead zone : arrêt des mouvements X et Y")
                            else:
                                # Calcul des vitesses
                                speed_x = -center_x_normalized
                                speed_y = -center_y_normalized

                            # Calcul de la vitesse Z
                            speed_z = max(0, (distance - min_target_distance) / max_target_distance)

                    # Encodage de l'image en format JPEG
                    _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 60])  # Compression plus rapide
                    img_bytes = buffer.tobytes()

                    # Conversion des données image en base64
                    img_base64 = base64.b64encode(img_bytes).decode('utf-8')

                    # Création des données à envoyer (vitesse + image encodée)
                    data = json.dumps({
                        "x": speed_x, 
                        "y": speed_y, 
                        "z": speed_z, 
                        "image": img_base64
                    })

                    # Envoi des données via WebSocket
                    print(f"Envoi des données : {data[:100]}...")  # Affichage d'un extrait des données
                    await websocket.send(data)
                    await asyncio.sleep(0.10)  # Fréquence d'envoi réduite à 10 Hz

        except Exception as e:
            print(f"Erreur de connexion WebSocket: {e}")
            print("Nouvelle tentative de connexion dans 3 secondes...")
            await asyncio.sleep(3)  # Attendre 3 secondes avant de réessayer

# Demander une valeur à l'utilisateur
SERVER_IP = "0.0.0.0" 
asyncio.run(send_data())