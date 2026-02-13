ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    cmake \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Create user
RUN useradd -m humble-dev && \
    echo "humble-dev:humble-dev" | chpasswd && adduser humble-dev sudo

USER humble-dev
WORKDIR /home/humble-dev

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/humble-dev/.bashrc