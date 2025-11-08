#!/bin/bash
# Check Nav2 Node Status
# Verifies all Nav2 lifecycle nodes are active and ready

echo "========================================"
echo "  BIN-BOY Nav2 Status Check"
echo "========================================"
echo ""

cd ~/bin-boy
source install/setup.bash

echo "Checking Nav2 lifecycle node states..."
echo ""

check_node() {
    local node=$1
    local state=$(ros2 lifecycle get $node 2>/dev/null | grep -oP '(?<=\[)[0-9]+(?=\])')
    local state_name=$(ros2 lifecycle get $node 2>/dev/null | grep -oP '^[a-z_]+(?= \[)')

    if [ "$state" == "3" ]; then
        echo "✓ $node: ACTIVE"
        return 0
    elif [ "$state" == "2" ]; then
        echo "⚠ $node: INACTIVE (needs activation)"
        return 1
    elif [ -z "$state" ]; then
        echo "✗ $node: NOT RUNNING"
        return 2
    else
        echo "? $node: $state_name [$state]"
        return 1
    fi
}

all_active=true

check_node "/controller_server" || all_active=false
check_node "/planner_server" || all_active=false
check_node "/recoveries_server" || all_active=false
check_node "/bt_navigator" || all_active=false
check_node "/waypoint_follower" || all_active=false

echo ""
echo "Checking costmap topics..."
ros2 topic hz /local_costmap/costmap_raw --window 10 --once 2>&1 | grep -q "average rate" && echo "✓ Local costmap publishing" || echo "✗ Local costmap not publishing"
ros2 topic hz /global_costmap/costmap_raw --window 10 --once 2>&1 | grep -q "average rate" && echo "✓ Global costmap publishing" || echo "✗ Global costmap not publishing"

echo ""
echo "Checking cmd_vel..."
ros2 topic info /cmd_vel | grep "Publisher count" | grep -q "1 " && echo "✓ Single /cmd_vel publisher" || echo "⚠ Multiple /cmd_vel publishers (conflict!)"

echo ""
if [ "$all_active" = true ]; then
    echo "========================================"
    echo "  ✓ Nav2 READY FOR NAVIGATION"
    echo "========================================"
    echo ""
    echo "Send navigation goals using:"
    echo "  - RViz: Click '2D Nav Goal' button"
    echo "  - CLI: ros2 action send_goal /navigate_to_pose ..."
else
    echo "========================================"
    echo "  ⚠ Nav2 NOT READY"
    echo "========================================"
    echo ""
    echo "If nodes are INACTIVE, wait 15 seconds after launch"
    echo "or manually activate with:"
    echo "  bash ~/bin-boy/activate_nav2.sh"
fi
echo ""
