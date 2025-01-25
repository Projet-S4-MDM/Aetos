# Aetos

## Prerequisites

### Docker installation

Set up Docker's apt repository.
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

Create the docker group.
   ```bash
sudo groupadd docker   
```

Add your user to the docker group.
   ```bash
 sudo usermod -aG docker $USER
 ```

 Once this is done, reboot your computer to enable changes to groups

### VSCode setup

Install the necessary vscode extenstions by running the following in your terminal:

   ```bash
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-vscode-remote.remote-ssh
```

## Getting Started

When VS Code detects the devcontainer configuration, you'll see a notification in the bottom-right corner. Click "Reopen in Container" to start the devcontainer.

   Alternatively, you can:
   - Press `Ctrl + Shift + P`
   - Type "Dev Containers: Reopen in Container"
   - Press Enter

## Development Inside the Container

Once inside the container, you'll have access to all dependencies specified in the Dockerfile. Here's what you need to know for development:

### Terminal Setup

To access the terminal in VSCode:
1. Press `Ctrl + Shift + '`
2. In the terminal, run:
   ```bash
   terminator
   ```

      to have access to the terminal multiplier. This facilitates development in the ros2 environment

     > **Note** The script that launches terminator currently runs into some issues. This does not affect usage

2. ROS2 development
   - The main branch currently contains a test_pgk which only serves as a template for git. Delete this package and replace it with your own. 
   > **Note** This will be removed as features will be merged
   - A ros2 package should be created for every different features. For example: there would be a ros2 package for vision recognition and another for a GUI.

## Developping in aetos_micro
