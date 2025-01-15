FROM osrf/ros:humble-desktop-full
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y

RUN apt-get update && sudo apt-get install -y \
    python3-pip \
    terminator

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

ENV SHELL /bin/bash
USER $USERNAME
CMD ["/bin/bash"]