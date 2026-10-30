# BlueROV2 Core

ROS 2 Jazzy localization and control package for BlueROV2.

## Quick Start (Docker)

1. Navigate to the docker directory and build the image:
   ```bash
   cd docker
   docker build -t ros2-dev .
   
2. Start the development container:
   ```bash
   ./run.sh

3. Inside the container, build and run:
   ```bash
   cd /root/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ros2 run bluerov_core localization


