#!/bin/bash
# Install all dependencies for simulation

set -e

echo "Installing ROS 2 Foxy dependencies for BIN-BOY simulation..."

# Core ROS packages
sudo apt-get update
sudo apt-get install -y \
  ros-foxy-xacro \
  ros-foxy-urdf \
  ros-foxy-robot-state-publisher \
  ros-foxy-joint-state-publisher \
  ros-foxy-joint-state-publisher-gui \
  ros-foxy-rviz2 \
  ros-foxy-rqt \
  ros-foxy-rqt-common-plugins

# Gazebo simulation
sudo apt-get install -y \
  ros-foxy-gazebo-ros \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-msgs \
  ros-foxy-gazebo-plugins

# Sensor fusion and localization
sudo apt-get install -y \
  ros-foxy-robot-localization

# Vision and perception
sudo apt-get install -y \
  ros-foxy-vision-msgs \
  ros-foxy-cv-bridge \
  ros-foxy-image-transport

# Navigation (for Phase 3 & 4)
sudo apt-get install -y \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox

# Python dependencies
sudo apt-get install -y \
  python3-pip \
  python3-numpy \
  python3-opencv

echo ""
echo "All dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. cd ~/bin-boy"
echo "2. colcon build --packages-select bin_boy_interfaces bin_boy_description bin_boy_simulation bin_boy_control bin_boy_perception"
echo "3. source install/setup.bash"
echo "4. ros2 launch bin_boy_simulation [ ].launch.py"
