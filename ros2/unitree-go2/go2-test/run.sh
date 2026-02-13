#!/bin/bash

container_image="go2-test:latest"
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

echo "Running container with:"
echo "  image:          $container_image"
echo "  ROS_DOMAIN_ID:  $ros_domain_id"

docker run \
  -it \
  --name go2-test \
  --rm \
  --net host \
  -v "$(pwd)/src:/home/go2/src:rw" \
  -e ROS_DOMAIN_ID="$ros_domain_id" \
  "$container_image"