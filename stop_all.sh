#!/bin/bash
# Clean shutdown of all ROS 2 and Gazebo processes

echo "========================================"
echo "  Stopping All ROS 2 & Gazebo Processes"
echo "========================================"
echo ""

echo "Sending SIGINT to ROS 2 processes..."
pkill -2 -f ros2
sleep 3

echo "Checking for remaining processes..."
if pgrep -f ros2 > /dev/null || pgrep -f gazebo > /dev/null || pgrep -f rviz2 > /dev/null; then
    echo "Some processes still running, sending SIGTERM..."
    pkill -15 -f ros2
    pkill -15 -f gazebo
    pkill -15 -f rviz2
    sleep 3
fi

if pgrep -f ros2 > /dev/null || pgrep -f gazebo > /dev/null || pgrep -f rviz2 > /dev/null; then
    echo "⚠ Forcefully killing remaining processes..."
    pkill -9 -f ros2
    pkill -9 -f gazebo
    pkill -9 -f rviz2
    sleep 1
fi

echo ""
echo "✓ All processes stopped"
echo ""
