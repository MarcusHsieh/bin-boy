#!/bin/bash
# Emergency kill and restart script for simulation

echo "===== BIN-BOY SIMULATION EMERGENCY RESTART ====="
echo ""
echo "Step 1: Killing all ROS 2 and Gazebo processes..."
pkill -9 -f ros2
pkill -9 -f gazebo
pkill -9 -f rviz2
sleep 3

echo ""
echo "Step 2: Cleaning environment..."
source /opt/ros/foxy/setup.bash
cd ~/bin-boy
source ~/bin-boy/install/setup.bash

echo ""
echo "Step 3: Verifying workspace..."
if ros2 pkg list | grep -q "bin_boy_simulation"; then
    echo "✓ Workspace sourced correctly"
else
    echo "✗ ERROR: Workspace not sourced!"
    echo "Run: cd ~/bin-boy && source install/setup.bash"
    exit 1
fi

echo ""
echo "Step 4: Ready to launch!"
echo ""
echo "============================================"
echo "NOW RUN IN SEPARATE TERMINALS:"
echo "============================================"
echo ""
echo "TERMINAL 1 (Gazebo + Robot + Sensors):"
echo "  cd ~/bin-boy && source install/setup.bash"
echo "  ros2 launch bin_boy_simulation full_system_sim.launch.py"
echo ""
echo "Wait for Gazebo to fully load, then:"
echo ""
echo "TERMINAL 2 (Person Detection + Tracking):"
echo "  cd ~/bin-boy && source install/setup.bash"
echo "  ros2 launch bin_boy_perception sim_person_tracking.launch.py enable_following:=true debug_logging:=true"
echo ""
echo "============================================"
echo ""
echo "Verify with:"
echo "  ros2 node list"
echo "  ros2 topic hz /person_detections"
echo "============================================"
