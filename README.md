# Aetos

## Prerequisites

### Docker Installation

Set up Docker's apt repository:

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \  
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \  
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \  
sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

To install the latest version, run:

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Create the Docker group:

```bash
sudo groupadd docker
```

Add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
```

Reboot your computer to enable changes to groups.


Reboot your computer to enable changes to groups.

---

### VS Code Setup

Install the necessary VS Code extensions by running the following command in your terminal:

```bash
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-vscode-remote.remote-ssh
```

---

## Getting Started

When VS Code detects the devcontainer configuration, you'll see a notification in the bottom-right corner. Click **"Reopen in Container"** to start the devcontainer.

Alternatively, you can:

1. Press `Ctrl + Shift + P`
2. Type "Dev Containers: Reopen in Container"
3. Press Enter

> **Note:** When opening for the first time or rebuilding the whole environment, this process is expected to take time. Sit back, relax and enjoy nonsense logs poping up in your terminal

---

## Development Inside the Container

Once inside the container, you'll have access to all dependencies specified in the Dockerfile. Here's what you need to know for development:

### Terminal Setup

To access the terminal in VS Code:

1. Press `Ctrl + Shift + ‘`
2. In the terminal, run:

```bash
terminator
```

This gives you access to the terminal multiplier, which facilitates development in the ROS2 environment.

> **Note:** The script that launches Terminator currently has some issues, but this does not affect usage.

---

### ROS2 Development

- The main branch currently contains a `test_pgk` package, which serves as a template for Git. Delete this package and replace it with your own.

  > **Note:** This package will be removed as features are merged.

- A ROS2 package should be created for each distinct feature. For example:
  - One package for vision recognition
  - Another package for a GUI

---

## Developing in `aetos_micro`

To access the `aetos_micro` development environment:

1. Open the folder as a PlatformIO project:
   - Click on the PlatformIO icon in your VS Code activity bar.
   - Click the "Pick folder" button.
2. When prompted, enter the following path:

```bash
/home/ws/aetos_micro/
```

Click "OK." This will reload the page and open only the `aetos_micro` folder in a PlatformIO project.

> **Note:** The Arduino include is currently unstable, which may cause issues with IntelliSense.

3. You can now code in the `aetos_micro` folder as in a regular PlatformIO environment.

4. To work in `aetos_micro` and `ros2_ws` simultaneously:
   - Open a VS Code terminal and enter the following commands:

```bash
terminator
cd ..
code .
```

## Raspberry Pi 5 Setup and Environment
The Raspberry Pi is powered by a 5V 3A power pack and runs on the Raspberry Pi 64-bit OS.
It's using a Raspberry Pi Camera V3 and communicates with the PC via a WebSocket server.
We use a WebSocket server because running Docker on the Raspberry Pi OS 64-bit has been difficult (impossible).
The problem has been resolved by simply sending the video feed and the command vector to the main PC using a WebSocket server.
We could have used Ubuntu instead of Raspbian, but Ubuntu doesn't have certain packages required for using the Raspberry Pi camera.

The Python script running on the Raspberry Pi detects the largest object of a certain color and sends a speed vector
oriented in the direction of the object. The magnitude of the vector varies between 0 → 1.

## Setup of the Raspberry Pi:
1. Install Raspberry Pi OS 64-bit:
[Download here](https://www.raspberrypi.com/software/)

3. Enable the camera:
```bash
sudo raspi-config
```
- Go to Interface Options` → `Camera` → `Enable`
- Reboot the Pi:
```bash
sudo reboot
```
3. Insall the required Python packages:
```bash
sudo apt update  
sudo apt install python3-pip  
pip install opencv-python websockets numpy picamera2
```
4. Download the Python script:
[Download here](https://github.com/Projet-S4-MDM/Aetos)

7. Set the server IP address in the script:
Open the script in an editor:
```bash
nano your_script_name.py
```
Change the line:
```bash
SERVER_IP = "0.0.0.0"  # Replace with your PC's IP address
```
6. Check your IP address in the terminal:
#### On Linux:
```bash
ip a
```
#### On Windows:
```bash
ipconfig
```
Look for `IPv4 Address`
7. Ensure your Raspberry Pi and PC are on the same Wi-Fi network.
### Running the Application
1. **On the PC (Server):**
- Start your WebSocket server by running the cam script in the àetos_cam` package.
- Open Docker in VScode and launch **Terminator** in the terminal:
```bash
terminator
```
- Navigate to the ROS2 workspace
```bash
cd /home/user/ros2_ws
```
- Buid the workspace:
```bash
b
```
- Run the Websocket server:
```bash
ros2 run aetos_cam camera
```
2. **You should see this in the terminal:**
```bash
Aetos@david-950QDB:/home/ws/ros2_ws$ ros2 run aetos_cam camera  
[WebSocket] Serveur démarré sur le port 8765  
[INFO] [1742396874.530350073] [data_publisher]: [ROS2] Publication: x=0.0, y=0.0, z=0.0  
```
3. **On the Raspberry Pi (Client):**

Start the Python script
```bash
python3 aetos_raspberry_Pi.py
```
### Auto-start the Python script on boot
1. Edit the `rc.local. file` :
```bash
sudo nano /etc/rc.local
```
2. Add the following line before the `exit 0`:
```bash
python3 /home/pi/<your-project-folder>/your_script_name.py &
```
- Replace `<your-project-folder>`and `your_script_name.py` with the actual path and filename
- the `&`at the end ensures the cript run in the background
3. Save and exit:
Press `CTRL + X`→ `Y` → `Enter`
4. Make `rc.local` executable (if it isn't already):
```bash
sudo chmod +x /etc/rc.local
```
5. Reboot the Raspberry Pi to verify:
```bash
sudo reboot
```
The script will now automatically stat on boot.

### Customization
1. **Change the color being detected:**
Modify the HSV range in the script:
```bash
lower_pink = np.array([30, 50, 50])  
upper_pink = np.array([90, 255, 255])  
```
2. **Adjust the dead zone size:**
Change the `dead_zone_size`parameter:
```bash
dead_zone_size = 0.2  # 20% of the image center
```
### Troubleshooting
If it doesn't work, here are some things you can check:

1. Check if the camera is connected and if your Raspberry Pi can display the video:
```bash
libcamera-hello
```
2. Verify that the IP address of your PC is correct and that you are connected to the same network.

3. Make sure the WebSocket server is running on your PC.

4. If it still doesn't work, good luck! ❤️

This will open the entire development environment in the same devcontainer but in a separate VS Code window.

## Developping in aetos_micro
