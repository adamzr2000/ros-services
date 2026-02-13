# ROS 2 distro as build arg (default: humble)
ARG ROS_DISTRO=humble

# Official ROS 2 base image
FROM ros:${ROS_DISTRO}

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
  && rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash go2 && \
    echo "go2:go2" | chpasswd && \
    apt-get update && apt-get install -y --no-install-recommends sudo && \
    adduser go2 sudo && \
    rm -rf /var/lib/apt/lists/*

USER go2
WORKDIR /home/go2

# Source ROS 2 environment for interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Default: open a shell (you can run rviz2 manually)
CMD ["rviz2"]
