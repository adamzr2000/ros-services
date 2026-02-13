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
    pkg-config \
    libgflags-dev \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
 && rm -rf /var/lib/apt/lists/*

# Create user
RUN useradd -m astra && \
    echo "astra:astra" | chpasswd && adduser astra sudo

# Build and install libuvc
WORKDIR /opt
RUN git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && mkdir -p build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && ldconfig

# ROS2 workspace

USER astra
WORKDIR /home/astra

RUN mkdir -p /home/astra/ros2_ws/src && \
    cd /home/astra/ros2_ws/src && \
    git clone https://github.com/orbbec/ros2_astra_camera.git && \
    cd /home/astra/ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

RUN echo "source /home/astra/ros2_ws/install/setup.bash" >> /home/astra/.bashrc

USER root
COPY rules/56-orbbec-usb.rules /etc/udev/rules.d/56-orbbec-usb.rules
RUN chmod 0644 /etc/udev/rules.d/56-orbbec-usb.rules
USER astra

CMD ["/bin/bash","-lc","source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/astra/ros2_ws/install/setup.bash && ros2 launch astra_camera astra.launch.xml"]