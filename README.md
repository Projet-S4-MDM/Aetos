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
   - Press `F1` or `Ctrl/Cmd + Shift + P`
   - Type "Dev Containers: Reopen in Container"
   - Press Enter

## What Happens Next

- VS Code will build the Docker container based on the configuration in `.devcontainer/devcontainer.json`
- The container will include all necessary dependencies and extensions
- Your workspace will reload inside the container
- You can start developing with a consistent environment

