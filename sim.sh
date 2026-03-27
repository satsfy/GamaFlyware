#!/bin/bash

pkill -f px4 || true
pkill -f "gz sim" || true
pkill -f gazebo || true
rm -f /tmp/px4-sock-* /tmp/px4-*.lock

WORLD_NAME=${1:-default}
GZ_SIM_RESOURCE_PATH=/sitl/gz/models:/sitl/gz/worlds

echo "Running Gazebo with depth camera support..."
echo "Using world: $WORLD_NAME"
echo "PX4_GZ_WORLDS: $PX4_GZ_WORLDS"
echo "GZ_SIM_RESOURCE_PATH: $GZ_SIM_RESOURCE_PATH"

cd PX4-Autopilot
PX4_GZ_WORLDS=/sitl/gz/worlds \
GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/sitl/gz/models:/sitl/gz/worlds:/usr/share/gz \
PX4_GZ_WORLD=$WORLD_NAME \
make px4_sitl gz_x500_depth
