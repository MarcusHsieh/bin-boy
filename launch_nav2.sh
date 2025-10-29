#!/bin/bash
# Launch SLAM + Nav2 Navigation Stack
# Ensures clean startup and automatic node activation

echo "========================================"
echo "  BIN-BOY SLAM + Nav2 Navigation Stack"
echo "========================================"
echo ""
echo "Launching:"
echo "  - Gazebo simulation"
echo "  - SLAM Toolbox (mapping)"
echo "  - Nav2 stack (navigation)"
echo "  - RViz visualization"
echo ""
echo "Nav2 nodes will auto-activate after 5 seconds..."
echo ""

cd ~/bin-boy
source install/setup.bash

ros2 launch bin_boy_navigation slam_navigation.launch.py
