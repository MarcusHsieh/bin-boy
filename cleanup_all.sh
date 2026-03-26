#!/bin/bash
# Complete ROS2/Gazebo/DDS Cleanup Script
# Kills all processes and cleans DDS discovery

echo "=========================================="
echo "Complete ROS2 System Cleanup"
echo "=========================================="

# Kill all ROS2 and Gazebo processes
echo "Stopping all ROS2 nodes..."
killall -9 gzserver gzclient rviz2 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ros2 run" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "amcl" 2>/dev/null
pkill -9 -f "map_server" 2>/dev/null
pkill -9 -f "slam_toolbox" 2>/dev/null
pkill -9 -f "controller_server" 2>/dev/null
pkill -9 -f "planner_server" 2>/dev/null
pkill -9 -f "bt_navigator" 2>/dev/null
pkill -9 -f "lifecycle_manager" 2>/dev/null
pkill -9 -f "person_tracker" 2>/dev/null
pkill -9 -f "mock_person_detector" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null

# Wait for processes to die
sleep 2

# Kill ROS2 daemon
echo "Stopping ROS2 daemon..."
pkill -9 -f "_ros2_daemon" 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 1

# Clean up shared memory (DDS)
echo "Cleaning DDS shared memory..."
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -rf /dev/shm/sem.* 2>/dev/null
rm -rf /tmp/.ros* 2>/dev/null

# Clear ROS logs (optional - uncomment if needed)
# echo "Clearing old ROS logs..."
# rm -rf ~/.ros/log/* 2>/dev/null

# Verify cleanup
REMAINING=$(ps aux | grep -E "ros2|gazebo|rviz|nav2|amcl|slam" | grep -v grep | grep -v cleanup_all | wc -l)

echo "=========================================="
if [ "$REMAINING" -eq 0 ]; then
    echo "All processes cleaned successfully"
else
    echo "Warning: $REMAINING processes still running"
    echo "Remaining processes:"
    ps aux | grep -E "ros2|gazebo|rviz|nav2|amcl|slam" | grep -v grep | grep -v cleanup_all
fi
echo "=========================================="
