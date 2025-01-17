FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Define the workspace path
ARG WORKSPACE_PATH=/home/${USERNAME}/workspace
ENV WORKSPACE=${WORKSPACE_PATH}

# Create the user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && chsh -s /bin/bash $USERNAME

# Install required packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ssh \
    curl \
    git \
    xterm \
    terminator \
    pipx \
    python3-pip \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*

# Switch to user
USER $USERNAME

# Set workspace as the working directory
WORKDIR ${WORKSPACE}

# Source ROS in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

# Default shell to bash
SHELL ["/bin/bash", "-c"]

# You can add additional commands that will run inside the workspace if needed
