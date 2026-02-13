#!/usr/bin/env bash

# Exit on error
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard