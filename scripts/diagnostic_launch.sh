#!/bin/bash
# Diagnostic Launch - Step by step component testing
# This will help identify exactly what's failing

set -e

echo "================================================"
echo "  DIAGNOSTIC LAUNCH TEST"
echo "================================================"
echo ""

MAP_FILE=~/bin-boy/maps/map_20251028_203407.yaml

# Verify map file exists
if [ ! -f "$MAP_FILE" ]; then
    echo "ERROR: Map file not found: $MAP_FILE"
    exit 1
fi
echo "✓ Map file exists: $MAP_FILE"
echo ""

cd ~/bin-boy
source install/setup.bash

# Launch step by step with diagnostics
echo "Launching system with verbose output..."
echo "Log: /tmp/diagnostic_launch.log"
echo ""

# Run launch with output to screen
ros2 launch bin_boy_navigation localization_navigation.launch.py \
    use_sim_time:=true \
    map:=$MAP_FILE 2>&1 | tee /tmp/diagnostic_launch.log
