#!/bin/bash
# Launch navigation with AMCL localization (uses saved map)
# Usage: bash launch_localization.sh [map_name]

set -e

MAP_NAME=${1:-current_map}
MAP_DIR=~/bin-boy/maps
MAP_FILE="$MAP_DIR/$MAP_NAME.yaml"

echo "========================================="
echo "  BIN-BOY Localization + Navigation"
echo "========================================="

# Check if map exists
if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    bash ~/bin-boy/scripts/list_maps.sh
    exit 1
fi

echo ""
echo "Map: $MAP_NAME"
echo "File: $MAP_FILE"
echo ""
echo "Launching:"
echo "  - Gazebo simulation"
echo "  - Map server (with saved map)"
echo "  - AMCL localization"
echo "  - Nav2 stack"
echo "  - RViz visualization"
echo ""
echo "After launch:"
echo "  1. Set initial pose in RViz (2D Pose Estimate)"
echo "  2. Send navigation goals (2D Nav Goal)"
echo ""
echo "========================================="

cd ~/bin-boy
source install/setup.bash

ros2 launch bin_boy_navigation localization_navigation.launch.py \
    use_sim_time:=true \
    map:=$MAP_FILE
