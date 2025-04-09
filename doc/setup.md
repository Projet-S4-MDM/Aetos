# Software setup

## Table of Contents
- [Prerequisites](#prerequisites)
- [Docker Installation](#docker-installation)
- [VS Code Setup](#vs-code-setup)
- [Raspberry Pi 5 Setup](#raspberry-pi-5-setup)  
  - [Setup of the Raspberry Pi](#setup-of-the-raspberry-pi)  
  - [Setup the Pi Camera](#setup-the-pi-camera)  
  - [IP Setup](#ip-setup)
- [You're now ready to use the software](#youre-now-ready-to-use-the-software)

## Prerequisites

The Aetos software runs on [Ubuntu 22.04](https://releases.ubuntu.com/jammy/). To facilitate dependency management, the project runs with [docker](https://www.docker.com/). The repository contains a custom image containing all necessary requirements for the project.

## Docker Installation

1. Set up Docker's apt repository:

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

**Reboot your computer to enable changes to groups.**

## VS Code Setup

Install the necessary VS Code extensions by running the following command in your terminal:

```bash
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-vscode-remote.remote-ssh
```

## Raspberry Pi 5 Setup
The Raspberry Pi is powered by a 5V 3A power pack and runs on the Raspberry Pi 64-bit OS. A [Raspberry Pi Camera V3](https://www.pishop.ca/product/raspberry-pi-camera-module-3/?src=raspberrypi) is being used to enable the vision recognition.

> **Note:** The Pi is currently independant from the docker DevContainer due to compatibility issues regarding the pi camera. The pi camera unfortunately does not run on Ubuntu 24.04 which is the only Ubuntu based operating system that can be flashed for the Pi5.

<!-- It's using a Raspberry Pi Camera V3 and communicates with the PC via a WebSocket server. -->
<!-- We use a WebSocket server because running Docker on the Raspberry Pi OS 64-bit has been difficult (impossible).
The problem has been resolved by simply sending the video feed and the command vector to the main PC using a WebSocket server.
We could have used Ubuntu instead of Raspbian, but Ubuntu doesn't have certain packages required for using the Raspberry Pi camera.

The Python script running on the Raspberry Pi detects the largest object of a certain color and sends a speed vector
oriented in the direction of the object. The magnitude of the vector varies between 0 → 1. -->

### Setup of the Raspberry Pi:
1. Install Raspberry Pi OS 64-bit: [Download here](https://www.raspberrypi.com/software/)

### Setup the Pi Camera
To enable the camera, open a terminal and enter the following:

```bash
sudo raspi-config
```
- Go to Interface Options` → `Camera` → `Enable`
- Reboot the Pi to enable changes:
```bash
sudo reboot
```
Insall the required Python packages:
```bash
sudo apt update  
sudo apt install python3-pip  
pip install opencv-python websockets numpy picamera2
```

### Network Setup
The Pi uses Websocket to communicate information to the main computer. To enable communication, ensure your Raspberry Pi and PC are on the same Wi-Fi network. Once both are on the same network, import [THIS FILE](doc/files/aetos_pi.py) to your raspberry pi. The following steps ensure the communication between your computer and the Pi.

1. Set the server IP address in the script:
Open the script in an editor:
```bash
nano aetos_pi.py # Replace the file name with your file name
```
Change the line:
```bash
SERVER_IP = "127.0.0.1"  # Replace with your PC's IP address
```

The Computer and Pi should now be ready for communication

## You're now ready to use the software
Follow the instructions in the [Usage README]((doc/usage.md)) to get started.


<!-- ### Running the Application
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

4. If it still doesn't work, good luck! ❤️ -->


