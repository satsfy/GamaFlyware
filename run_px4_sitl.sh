#!/bin/bash

# # If sourced (for shell completion), do not execute SITL.
# if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
#     _run_px4_sitl_completion() {
#         local cur opts
#         COMPREPLY=()
#         cur="${COMP_WORDS[COMP_CWORD]}"
#         opts=$(ls /sitl/gz/worlds/*.sdf 2>/dev/null | xargs -n1 basename | sed 's/\.sdf$//')
#         COMPREPLY=( $(compgen -W "${opts}" -- "${cur}") )
#     }
#     complete -F _run_px4_sitl_completion run_px4_sitl.sh
#     return 0
# fi

if [[ -z "${1:-}" ]]; then
    echo "Please specify a world."
    # echo "Available worlds:"
    # ls /sitl/gz/worlds/*.sdf 2>/dev/null | xargs -n1 basename | sed 's/\.sdf$//' | sed 's/^/- /'
    echo -e "Tip: use TAB completion \n\n\t./run_px4_sitl.sh <TAB> \n"
    exit 1
fi

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
