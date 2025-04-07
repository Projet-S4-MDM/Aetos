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

The dockfile has Terminator installed which is a terminal demultiplier. This facilitates development especially for a ros2 project. To access terminator, open a VSCode terminal and enter the following. 

```bash
terminator
```
---