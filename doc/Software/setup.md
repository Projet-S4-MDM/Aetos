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

### 1. Install Raspberry Pi OS (64-bit)

Download the OS from the official Raspberry Pi website:  
[Download Raspberry Pi OS](https://www.raspberrypi.com/software/)

### 2. Setup the Pi Camera

1. Open the terminal:
   ```bash
   sudo raspi-config
   ```
2. Navigate to:  
   `Interface Options` → `Camera` → `Enable`

3. Reboot the Pi to apply changes:
   ```bash
   sudo reboot
   ```

4. Install required Python packages:
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip install opencv-python websockets numpy picamera2
   ```

### 3. Network Setup

The Raspberry Pi communicates with the main computer via WebSocket. Make sure both devices are on the **same Wi-Fi network**.

1. **Transfer the communication script** to your Raspberry Pi. Find the script [HERE](../Software/Files/aetos_pi.py)

2. **Edit the script to set your PC's IP address:**
   ```bash
   nano aetos_pi.py
   ```
   Locate the line:
   ```python
   SERVER_IP = "127.0.0.1"
   ```
   Replace `127.0.0.1` with your PC’s local IP address (e.g., `192.168.x.x`)

### 5. Auto-Start Script on Boot

1. Open `rc.local`:
   ```bash
   sudo nano /etc/rc.local
   ```

2. Add the following line **before** `exit 0`:
   ```bash
   python3 /home/pi/<your-project-folder>/your_script_name.py &
   ```
   - Replace `<your-project-folder>` and `your_script_name.py` with the actual path and file name.
   - The `&` ensures the script runs in the background.

3. Save and exit:  
   Press `CTRL + X`, then `Y`, then `Enter`.

4. Make sure `rc.local` is executable:
   ```bash
   sudo chmod +x /etc/rc.local
   ```

5. Reboot to verify:
   ```bash
   sudo reboot
   ```


**Done!** Your Raspberry Pi should now start the camera communication script automatically on boot and be ready to send data to your PC.

## You're now ready to use the software
Follow the instructions in the [Usage README](usage.md) to get started.



