#!/bin/bash

CONFIG_FILE="/home/go2/ros2_ws/src/go2_base/config/go2_base.yaml"

TARGET_PORT=${GO2_LAN_PORT:-"enp92s0"}

echo "Configuring YAML: Setting lan_port to '$TARGET_PORT' in $CONFIG_FILE"

sed -i "s/lan_port: .*/lan_port: '$TARGET_PORT'/" "$CONFIG_FILE"

source /opt/ros/humble/setup.bash
source /home/go2/ros2_ws/install/setup.bash

exec ros2 launch go2_bringup system.launch.py