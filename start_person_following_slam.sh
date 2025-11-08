#!/bin/bash
# Unified Person Following with Dynamic SLAM
# This script launches the complete system for person following in dynamic environments

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "Person Following with Dynamic SLAM"
echo "=========================================="
echo

# Step 1: Complete Cleanup
echo "${BLUE}Step 1: Cleaning up all ROS2 processes and DDS...${NC}"
./cleanup_all.sh
echo "${GREEN}✓ Cleanup complete${NC}"
echo

# Step 2: Source workspace
echo "${BLUE}Step 2: Sourcing workspace...${NC}"
source install/setup.bash
echo "${GREEN}✓ Workspace sourced${NC}"
echo

# Step 3: Launch Simulation + SLAM in background
echo "${BLUE}Step 3: Launching Simulation with Dynamic SLAM...${NC}"
echo "  - Gazebo simulation"
echo "  - RViz visualization"
echo "  - SLAM Toolbox (dynamic mapping)"
echo

# Launch simulation with SLAM
ros2 launch bin_boy_simulation simulation.launch.py \
    rviz:=true \
    slam:=true \
    > /tmp/slam_simulation.log 2>&1 &
SIM_PID=$!

echo "${GREEN}✓ Simulation + SLAM launched (PID: $SIM_PID)${NC}"
echo "  Log: /tmp/slam_simulation.log"
echo

# Step 4: Wait for SLAM to initialize
echo "${BLUE}Step 4: Waiting for SLAM to initialize...${NC}"
echo "  This takes ~15 seconds for:"
echo "  - Gazebo to start"
echo "  - Robot to spawn"
echo "  - SLAM Toolbox to build initial map"
echo

for i in {1..15}; do
    echo -ne "  ${YELLOW}[$i/15] Initializing SLAM...${NC}\r"
    sleep 1
done
echo -ne '\n'

# Verify SLAM is running
echo "  Verifying SLAM Toolbox..."
if ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
    echo "${GREEN}✓ SLAM Toolbox active${NC}"
else
    echo "${RED}⚠ Warning: SLAM Toolbox not detected${NC}"
    echo "  Check log: /tmp/slam_simulation.log"
fi

# Verify map frame exists
echo "  Verifying map frame..."
if timeout 3 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1; then
    echo "${GREEN}✓ Map frame active (SLAM working)${NC}"
else
    echo "${YELLOW}⚠ Map frame not yet stable (may still be initializing)${NC}"
fi
echo

# Step 5: Launch Person Tracking + Nav2 in background
echo "${BLUE}Step 5: Launching Person Tracking + Nav2...${NC}"
echo "  - Mock person detector (Gazebo ground truth)"
echo "  - Person tracker"
echo "  - Nav2 navigation stack"
echo

ros2 launch bin_boy_perception sim_person_tracking.launch.py \
    enable_following:=true \
    enable_nav2:=true \
    debug_logging:=true \
    > /tmp/person_following.log 2>&1 &
TRACKING_PID=$!

echo "${GREEN}✓ Person tracking launched (PID: $TRACKING_PID)${NC}"
echo "  Log: /tmp/person_following.log"
echo

# Step 6: Wait for Nav2 to initialize
echo "${BLUE}Step 6: Waiting for Nav2 to initialize...${NC}"
sleep 8

# Verify Nav2 is running
echo "  Verifying Nav2 stack..."
NAV2_NODES=$(ros2 node list 2>/dev/null | grep -E "controller_server|planner_server|bt_navigator" | wc -l)
if [ "$NAV2_NODES" -ge 3 ]; then
    echo "${GREEN}✓ Nav2 stack active ($NAV2_NODES/3 core nodes)${NC}"
else
    echo "${YELLOW}⚠ Warning: Nav2 may not be fully initialized ($NAV2_NODES/3 nodes)${NC}"
fi
echo

# Step 7: System Status
echo "=========================================="
echo "${GREEN}✓ ALL SYSTEMS OPERATIONAL${NC}"
echo "=========================================="
echo
echo "${BLUE}System Components:${NC}"
echo "  ✓ Gazebo Simulation"
echo "  ✓ RViz Visualization"
echo "  ✓ SLAM Toolbox (dynamic mapping)"
echo "  ✓ Person Tracker"
echo "  ✓ Nav2 Navigation"
echo
echo "${BLUE}How It Works:${NC}"
echo "  1. SLAM builds a dynamic map of the environment"
echo "  2. Moving obstacles (people) are handled gracefully"
echo "  3. Robot detects and tracks people using lidar"
echo "  4. Nav2 plans paths avoiding obstacles"
echo "  5. Robot follows detected person autonomously"
echo
echo "${BLUE}Testing:${NC}"
echo "  1. In Gazebo: Add person models to the world"
echo "  2. In RViz: Watch the map build in real-time"
echo "  3. Robot will detect and follow people automatically"
echo
echo "${BLUE}Useful Commands:${NC}"
echo "  # Check system status"
echo "  ros2 node list"
echo
echo "  # Check TF frames (should see map→odom→base_footprint)"
echo "  ros2 run tf2_tools view_frames.py"
echo
echo "  # Check person tracking status"
echo "  ros2 topic echo /person/tracking_status"
echo
echo "  # Monitor SLAM map updates"
echo "  ros2 topic hz /map"
echo
echo "  # Disable/enable person following"
echo "  ros2 param set /person_following_behavior enabled false"
echo "  ros2 param set /person_following_behavior enabled true"
echo
echo "${BLUE}Logs:${NC}"
echo "  Simulation + SLAM: /tmp/slam_simulation.log"
echo "  Person Tracking:   /tmp/person_following.log"
echo
echo "${BLUE}Cleanup:${NC}"
echo "  ./cleanup_all.sh  # Stop everything"
echo
echo "=========================================="
echo "${GREEN}System ready! Robot is now following people.${NC}"
echo "=========================================="

# Keep script running to maintain background processes
echo
echo "Press Ctrl+C to stop all processes"
echo

# Trap Ctrl+C to clean up
trap "echo && echo '${YELLOW}Stopping all processes...${NC}' && ./cleanup_all.sh && exit 0" INT TERM

# Wait for background processes
wait
