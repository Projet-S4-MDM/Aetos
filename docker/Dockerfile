FROM osrf/ros:humble-desktop-full

# Set non-interactive mode to avoid prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Set workspace path
ARG USERNAME=Aetos
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE_PATH=/home/${USERNAME}/workspace
ENV WORKSPACE=${WORKSPACE_PATH}

# Create user and set permissions
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG plugdev,dialout $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && chsh -s /bin/bash $USERNAME

# Install required dependencies
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ssh \
    curl \
    git \
    terminator \
    pipx \
    nano \
    udev \
    python3-pip \
    python3-transforms3d \
    libnotify-bin \
    dbus-x11 \
    at-spi2-core \
    python3-notify2 \
    bluez \
    bluez-tools \
    bluetooth \
    dbus \
    libbluetooth-dev \
    libeigen3-dev \
    libserial-dev \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    qttools5-dev-tools \
    qttools5-dev \
    python3-pyqt5 \
    python3-pyqt5.qtmultimedia \
    python3-pyqt5.qtquick \
    qml-module-qtquick-controls \
    qml-module-qtquick-dialogs \
    qml-module-qtquick-layouts \
    qml-module-qtgraphicaleffects \
    qml-module-qtquick-window2 \
    qml-module-qtquick2 \
    && rm -rf /var/lib/apt/lists/*

# Create DBus directory and set permissions
RUN mkdir -p /var/run/dbus && chmod 755 /var/run/dbus

# Install Python packages
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        pygame \
        setuptools==58.2.0 \
        evdev \
        pybullet \
        websockets \
        matplotlib

# Set up environment for ROS2
USER $USERNAME
WORKDIR ${WORKSPACE}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /home/ws/ros2_ws/install/local_setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=69" >> /home/${USERNAME}/.bashrc && \
    echo "alias b='pushd . > /dev/null && cd /home/ws/ros2_ws && colcon build --symlink-install && popd > /dev/null'" >> /home/${USERNAME}/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc

# Install PlatformIO udev rules
RUN curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Use bash shell
SHELL ["/bin/bash", "-c"]
