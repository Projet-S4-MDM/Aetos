import asyncio
import websockets
import json
import base64
import cv2
import numpy as np

# Variable pour stocker les données WebSocket
data = {"x": 0.0, "y": 0.0, "z": 0.0}
frame = None  # Variable pour stocker l'image décodée

# Fonction pour décoder l'image base64
def decode_image(base64_str):
    img_data = base64.b64decode(base64_str)
    np_array = np.frombuffer(img_data, np.uint8)
    image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
    return image

# Fonction pour gérer les connexions WebSocket
async def handle_connection(websocket):
    global data, frame
    async for message in websocket:
        # Charger les données reçues
        received_data = json.loads(message)
        print(f"Received: X: {received_data['x']}, Y: {received_data['y']}, Z: {received_data['z']}")

        # Sauvegarder les données de vitesse dans un fichier JSON
        data = {
            "x": received_data['x'],
            "y": received_data['y'],
            "z": received_data['z']
        }
        with open("data.json", "w") as f:
            json.dump(data, f)

        # Vérifier si l'image est présente dans les données
        if 'image' in received_data:
            img_base64 = received_data['image']
            frame = decode_image(img_base64)  # Décoder l'image

            # Afficher l'image reçue (facultatif)
            if frame is not None:
                cv2.imshow("Received Image", frame)
                cv2.waitKey(1)

# Fonction pour démarrer le serveur WebSocket
async def start_websocket_server():
    server = await websockets.serve(handle_connection, "0.0.0.0", 8765)
    print("Serveur WebSocket démarré sur le port 8765")
    await server.wait_closed()

# Lancer le serveur WebSocket
def run_server():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_websocket_server())


def main():
    run_server()
if __name__ == "__main__":
    main()
