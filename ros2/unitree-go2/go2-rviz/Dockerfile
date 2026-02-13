ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System + build tools + rosdep + colcon + rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    git \
  && rm -rf /var/lib/apt/lists/*

# rosdep init/update (needs root)
RUN rosdep init || true && rosdep update

# Create non-root user (optional)
RUN useradd -m -s /bin/bash go2 && \
    echo "go2:go2" | chpasswd

WORKDIR /home/go2

# Copy workspace
RUN mkdir -p /home/go2/ros2_ws/src
COPY src /home/go2/ros2_ws/src

# Install deps + build (run as root so rosdep can apt-get)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install --from-paths /home/go2/ros2_ws/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    cd /home/go2/ros2_ws && \
    colcon build --symlink-install

# Switch to user for runtime
RUN chown -R go2:go2 /home/go2
USER go2

# Source overlays in bash sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/go2/.bashrc && \
    echo "source /home/go2/ros2_ws/install/setup.bash" >> /home/go2/.bashrc

CMD ["bash", "-lc", "source /home/go2/ros2_ws/install/setup.bash && ros2 launch go2_viz view_robot.launch.py"]
