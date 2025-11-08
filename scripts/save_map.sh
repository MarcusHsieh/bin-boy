#!/bin/bash
# Save current SLAM map with timestamp
set -e

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MAP_DIR=~/bin-boy/maps
MAP_NAME=${1:-map_$TIMESTAMP}

mkdir -p $MAP_DIR

echo "========================================="
echo "  Saving SLAM Map"
echo "========================================="
echo "Map name: $MAP_NAME"
echo "Location: $MAP_DIR"

# Save map using map_saver_cli
cd $MAP_DIR
ros2 run nav2_map_server map_saver_cli -f $MAP_NAME --ros-args -p use_sim_time:=true

if [ -f "$MAP_NAME.yaml" ] && [ -f "$MAP_NAME.pgm" ]; then
    echo ""
    echo "✓ Map saved successfully!"
    echo "  YAML: $MAP_DIR/$MAP_NAME.yaml"
    echo "  PGM:  $MAP_DIR/$MAP_NAME.pgm"

    # Create metadata file
    # Get map size from PGM file (line 2 contains width x height)
    MAP_SIZE=$(sed -n '2p' "$MAP_NAME.pgm" | awk '{print $1 "x" $2 " pixels"}')

    cat > "$MAP_NAME.metadata.txt" << EOF
Saved: $(date)
Resolution: $(grep resolution "$MAP_NAME.yaml" | awk '{print $2}')
Origin: $(grep origin "$MAP_NAME.yaml")
Map size: $MAP_SIZE
EOF

    echo ""
    echo "To use this map for localization:"
    echo "  bash ~/bin-boy/scripts/load_map.sh $MAP_NAME"
else
    echo "✗ Map save failed!"
    exit 1
fi
