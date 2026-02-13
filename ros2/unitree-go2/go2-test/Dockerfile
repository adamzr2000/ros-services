ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    sudo \
  && rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash go2 && \
    echo "go2:go2" | chpasswd && \
    adduser go2 sudo

USER go2
WORKDIR /home/go2

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/go2/.bashrc

CMD ["bash"]
