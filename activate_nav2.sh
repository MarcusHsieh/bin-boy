#!/bin/bash
# Manually Activate Nav2 Nodes
# Use this if autostart failed or nodes are stuck in INACTIVE state

echo "========================================"
echo "  Manually Activating Nav2 Nodes"
echo "========================================"
echo ""

cd ~/bin-boy
source install/setup.bash

activate_node() {
    local node=$1
    echo "Activating $node..."
    ros2 lifecycle set $node configure 2>&1 | grep -q "Transitioning successful" && \
    ros2 lifecycle set $node activate 2>&1 | grep -q "Transitioning successful" && \
    echo "  ✓ $node activated" || echo "  ✗ $node activation failed"
}

activate_node "/controller_server"
activate_node "/planner_server"
activate_node "/recoveries_server"
activate_node "/bt_navigator"
activate_node "/waypoint_follower"

echo ""
echo "========================================"
echo "  Activation Complete"
echo "========================================"
echo ""
echo "Verify status with:"
echo "  bash ~/bin-boy/check_nav2_status.sh"
echo ""
