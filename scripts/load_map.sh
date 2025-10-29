#!/bin/bash
# Load a saved map for localization
set -e

MAP_DIR=~/bin-boy/maps
MAP_NAME=${1:-current_map}
MAP_FILE="$MAP_DIR/$MAP_NAME.yaml"

if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map file not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    bash ~/bin-boy/scripts/list_maps.sh
    exit 1
fi

echo "========================================="
echo "  Loading Map for Localization"
echo "========================================="
echo "Map: $MAP_NAME"
echo "File: $MAP_FILE"
echo ""

# Update nav2_params.yaml with map filename
sed -i "s|yaml_filename:.*|yaml_filename: \"$MAP_FILE\"|" \
    ~/bin-boy/src/bin_boy_navigation/config/nav2_params.yaml

echo "✓ Map configured in nav2_params.yaml"
echo ""
echo "To launch with this map:"
echo "  bash ~/bin-boy/launch_localization.sh"
