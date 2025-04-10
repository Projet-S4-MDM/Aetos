# Usage

## Opening the Devcontainer

When VS Code detects the devcontainer configuration, you'll see a notification in the bottom-right corner. Click **"Reopen in Container"** to start the devcontainer.

Alternatively, you can:

1. Press `Ctrl + Shift + P`
2. Type "Dev Containers: Reopen in Container"
3. Press Enter

> **Note:** When opening for the first time or rebuilding the whole environment, this process is expected to take time. Sit back, relax and enjoy nonsense logs poping up in your terminal


## Development Inside the Container

Once inside the container, you'll have access to all dependencies specified in the Dockerfile. Here's what you need to know for development:

The dockfile has Terminator installed which is a terminal demultiplier. This facilitates development especially for a ros2 project. To access terminator, open a VSCode terminal and enter the following. 

```bash
terminator
```
The project's  `.bashrc` is equiped with a command which allows you to always build the ros2 workspace in the right folder using the command: "b" from a terminal. 

## Run the code

To launch the application, you can simply open a VSCode terminal and type:

```bash
ros2 launch aetos_auxiliary aetos.launch.py
```
This will launch all necessary nodes to controle Aetos as well as the GUI. Once the GUI is launched it serves as a visualiser for the camera feed as well as a control for the input command arbitration. Note that no motor commands will be sent without the computer being connected through a serial port to the ESP32.

## Simulation

Aetos is also equiped with a simulation for development purposes. If you want to to launch the code in simulation mode, run the following in a VsCode temrinal:

```bash
ros2 launch aetos_auxiliary aetos.launch.py PARAMETER
```

> **Note:** The default parameter can be switched to simulation for development purposes

## Controls

Aetos is controlled using a bluetooth controller. The following keybindings can be modified in the `joy_demux.py` file from the `aetos_joy` ros2 package.

![Controller](../images/controls.png)

## Homing

To avoid a error accumulation and drift, Aetos requires homing everytime it is restarted. To perform homing, simply place the payload on the location provided for this purpose (see [Mechanical Assembly](../Mechanical/mecanical_assembly.md) for more details). Once at the desired position, you can now reset the encoders using the assigned controls. 

>  **Caution:** Make sure the motor power is **disconnected** before resetting the encoders. Failing to do so may lead to unintended motion or damage.

## Graphical Interface

## Emergency stop

## Vision

To enable vision, make sure all nodes have been correctly launched. If all a connection has been successfully established between the main computer and the rasperry pi, you should be able to see the camera feed appear on the GUI. You can then toggle bewtween Teleop control and autonomous control using the buttons.

>  **Note:** Many of the current elements of vision are  modifiable in 
172.20.10.3


