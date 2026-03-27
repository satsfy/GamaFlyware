#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
cd PX4-Autopilot
source /opt/ros/humble/setup.bash
make px4_sitl gz_x500
exec "$@"
