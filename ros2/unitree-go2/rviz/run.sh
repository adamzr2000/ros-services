#!/bin/bash
set -e

container_image="rviz2:latest"
ros_domain_id=0
rviz_config=""  # optional

host_config_dir="$(pwd)/config"
container_config_dir="/home/go2/config"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-domain-id)
      ros_domain_id="$2"
      shift 2
      ;;
    --rviz-config)
      rviz_config="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--ros-domain-id <ros_domain_id>] [--rviz-config <file.rviz>]"
      echo "Defaults: ROS_DOMAIN_ID=0"
      echo "RViz configs must be under: ./config (mounted to ${container_config_dir})"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--ros-domain-id <ros_domain_id>] [--rviz-config <file.rviz>]"
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

# Ensure config dir exists so the mount always works
mkdir -p "$host_config_dir"

# Optional rviz args (only allow files from ./config)
rviz_args=()
if [[ -n "$rviz_config" ]]; then
  # Allow "foo.rviz" or "config/foo.rviz"
  rel="${rviz_config#config/}"

  # Disallow absolute paths and traversal
  if [[ "$rviz_config" == /* || "$rel" == *".."* ]]; then
    echo "ERROR: --rviz-config must point to a file under ./config."
    exit 1
  fi

  host_rviz_path="${host_config_dir}/${rel}"
  if [[ ! -f "$host_rviz_path" ]]; then
    echo "ERROR: RViz config not found: $host_rviz_path"
    exit 1
  fi

  rviz_args+=("-d" "${container_config_dir}/${rel}")
fi

echo "Running container with:"
echo "  image:          $container_image"
echo "  ROS_DOMAIN_ID:  $ros_domain_id"
echo "  config mount:   $host_config_dir -> $container_config_dir"
if [[ -n "$rviz_config" ]]; then
  echo "  rviz config:    ${rviz_config#config/}"
fi

docker run \
  -it \
  --name rviz2 \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="QT_QPA_PLATFORM=xcb" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --rm \
  --net host \
  -e ROS_DOMAIN_ID="$ros_domain_id" \
  --privileged \
  -v "${host_config_dir}:${container_config_dir}:rw" \
  "$container_image" \
  "${rviz_args[@]}"
