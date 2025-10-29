#!/bin/bash
# Test Localization Launch
# Verifies map server, AMCL, and Nav2 are working correctly

set -e

MAP_NAME=${1:-map_20251028_203407}
TEST_LOG="/tmp/localization_test_$(date +%s).log"

echo "=================================="
echo "  Localization System Test"
echo "=================================="
echo ""
echo "Map: $MAP_NAME"
echo "Log: $TEST_LOG"
echo ""

# Launch in background
echo "[1/6] Launching localization system..."
cd ~/bin-boy
source install/setup.bash
ros2 launch bin_boy_navigation localization_navigation.launch.py \
    use_sim_time:=true \
    map:=~/bin-boy/maps/$MAP_NAME.yaml \
    > "$TEST_LOG" 2>&1 &
LAUNCH_PID=$!

echo "Launch PID: $LAUNCH_PID"
echo "Waiting for system to initialize..."
sleep 15

# Check if launch is still running
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo "ERROR: Launch process died!"
    echo "Check log: $TEST_LOG"
    exit 1
fi
echo "✓ Launch process running"
echo ""

# Test 1: Check if map_server is running
echo "[2/6] Checking map_server..."
if ros2 node list | grep -q "map_server"; then
    echo "✓ map_server node running"
else
    echo "✗ map_server not found!"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# Test 2: Check if map is being published
echo "[3/6] Checking map topic..."
timeout 5 ros2 topic echo /map --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✓ Map is being published on /map"
else
    echo "✗ Map not available on /map topic!"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# Test 3: Check if AMCL is running
echo "[4/6] Checking AMCL..."
if ros2 node list | grep -q "amcl"; then
    echo "✓ AMCL node running"
else
    echo "✗ AMCL not found!"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# Test 4: Check if controller_server is running
echo "[5/6] Checking controller_server..."
if ros2 node list | grep -q "controller_server"; then
    echo "✓ controller_server running"
else
    echo "✗ controller_server not found!"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

# Test 5: Check if planner_server is running
echo "[6/6] Checking planner_server..."
if ros2 node list | grep -q "planner_server"; then
    echo "✓ planner_server running"
else
    echo "✗ planner_server not found!"
    kill $LAUNCH_PID 2>/dev/null
    exit 1
fi

echo ""
echo "=================================="
echo "  All Tests Passed! ✓"
echo "=================================="
echo ""
echo "System is running correctly."
echo "Launch PID: $LAUNCH_PID"
echo "Log file: $TEST_LOG"
echo ""
echo "NEXT STEPS:"
echo "1. Open RViz (should already be open)"
echo "2. Click '2D Pose Estimate' button"
echo "3. Click on map where robot is located"
echo "4. Drag to set orientation"
echo "5. Then click '2D Nav Goal' to test navigation"
echo ""
echo "To stop: kill $LAUNCH_PID"
echo ""

# Keep script running so user can see output
read -p "Press Enter to shutdown system..." dummy
kill $LAUNCH_PID 2>/dev/null
echo "System shutdown complete"
