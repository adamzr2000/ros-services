#!/bin/bash
set -e

container_image="go2-rviz:latest"
ros_domain_id=0

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-domain-id)
      ros_domain_id="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--ros-domain-id <ros_domain_id>]"
      echo "Defaults: ROS_DOMAIN_ID=0"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--ros-domain-id <ros_domain_id>]"
      exit 1
      ;;
  esac
done

# X11 access (host must have xhost installed)
if command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null
else
  echo "WARNING: xhost not found. Install: sudo apt-get install -y x11-xserver-utils"
fi

echo "Running container with:"
echo "  image:          $container_image"
echo "  ROS_DOMAIN_ID:  $ros_domain_id"

docker run \
  -it \
  --name go2-rviz \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="QT_QPA_PLATFORM=xcb" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --rm \
  --net host \
  -e ROS_DOMAIN_ID="$ros_domain_id" \
  --privileged \
  "$container_image"
