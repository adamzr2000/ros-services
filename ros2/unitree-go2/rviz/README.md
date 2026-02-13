# RViz (ROS Visualization)

## What is it?

This module provides [Rviz2](https://docs.ros.org/en/humble/p/rviz2/), a 3D visualization tool for ROS 2, used to display data in real time such as robot state, TF frames, sensor outputs, and maps.

The graphical interface is displayed on the host machine using native X11 forwarding.

## What does this container do?

- Runs **RViz2**
- Uses **X11 forwarding** to display the GUI on the host
- Supports **ROS_DOMAIN_ID** selection
- Allows loading **RViz config files** from a shared `./config` directory

## Usage

### Dependencies

- `xhost` (`x11-xserver-utils` package)

On Ubuntu:
```bash
sudo apt-get install -y x11-xserver-utils
```

### Build 

```bash
./build.sh
```

### Run

Start RViz2:
```bash
./run.sh
```

Set a ROS domain ID:
```bash
./run.sh --ros-domain-id 7
```

Load an RViz configuration file (must be in `./config`):
```bash
./run.sh --rviz-config my_config.rviz
```

### How to use it?

RViz2 starts automatically when the container is launched.
If a configuration file is provided, it is loaded on startup.