FROM osrf/ros:foxy-ros1-bridge

ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-c" ]

# Fix the expired GPG keys before updating
RUN apt-get update || true && apt-get install -y curl && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Now the update and install will work
RUN apt-get update && apt-get install -y \
    ros-foxy-compressed-image-transport \
    ros-noetic-compressed-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

CMD ["ros2", "run", "ros1_bridge", "dynamic_bridge", "--bridge-all-topics"]