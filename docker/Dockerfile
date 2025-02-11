FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

ARG WORKSPACE_PATH=/home/${USERNAME}/workspace
ENV WORKSPACE=${WORKSPACE_PATH}

ARG USERNAME=Aetos
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && chsh -s /bin/bash $USERNAME

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ssh \
    curl \
    git \
    terminator \
    pipx \
    nano \
    python3-pip \
    python3-transforms3d \
    libnotify-bin \
    dbus-x11 \
    at-spi2-core \
    python3-notify2 \
    libeigen3-dev \  
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

WORKDIR ${WORKSPACE}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \ 
    echo "source /home/ws/ros2_ws/install/local_setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=69" >> /home/${USERNAME}/.bashrc && \
    echo "alias b='pushd . > /dev/null && cd /home/ws/ros2_ws && colcon build --symlink-install && popd > /dev/null'" >> /home/${USERNAME}/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/${USERNAME}/.bashrc

SHELL ["/bin/bash", "-c"]
