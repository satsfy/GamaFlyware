#!/bin/bash

WORLD_NAME=${1:-default}

cd PX4-Autopilot

PX4_GZ_WORLDS=/sitl/gz/worlds GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/sitl/gz/models:/sitl/gz/worlds:/usr/share/gz PX4_GZ_WORLD=$WORLD_NAME make px4_sitl gz_x500_depth

