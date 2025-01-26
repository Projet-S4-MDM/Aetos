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

This will open the entire development environment in the same devcontainer but in a separate VS Code window.

## Developping in aetos_micro
