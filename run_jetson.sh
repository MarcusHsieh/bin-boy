#!/bin/bash

SERIAL_PORT="/dev/ttyUSB0"
CAMERA_PKG="csi_camera_cpp"
CAMERA_LAUNCH_FILE="csi_camera_ipc.launch.py"
MOTOR_CONTROLLER_PKG="motor_controller"
MOTOR_CONTROLLER_NODE="twist_to_serial"

ROS2_WORKSPACE_PATH=~/bin-boy

if [ -f /opt/ros/foxy/setup.bash ]; then
  echo "Sourcing ROS 2 Foxy setup..."
  source /opt/ros/foxy/setup.bash
else
  echo "Error: ROS 2 Foxy main setup script not found!"
  exit 1
fi

if [ -f "${ROS2_WORKSPACE_PATH}/install/setup.bash" ]; then
  echo "Sourcing local workspace setup (${ROS2_WORKSPACE_PATH})..."
  source "${ROS2_WORKSPACE_PATH}/install/setup.bash"
else
  echo "Warning: Local workspace setup script not found at ${ROS2_WORKSPACE_PATH}/install/setup.bash"
  echo "Ensure you have built your workspace (colcon build)."
fi

echo "--- Starting Nodes ---"

cleanup() {
    echo "Caught Signal... Shutting down background processes."
    kill -SIGINT 0
    wait
    echo "Cleanup complete."
}

trap cleanup SIGINT SIGTERM

# launch camera node in the background (publish is false rn)
echo "Launching camera node: ${CAMERA_PKG} ${CAMERA_LAUNCH_FILE}..."
ros2 launch ${CAMERA_PKG} ${CAMERA_LAUNCH_FILE} detection_frame_skip:=4 publish_annotated_image:=false &
CAMERA_PID=$!
echo "Camera node started with PID: ${CAMERA_PID}"

# launch motor controller node in the background
echo "Launching motor controller node: ${MOTOR_CONTROLLER_PKG} ${MOTOR_CONTROLLER_NODE}..."
ros2 run ${MOTOR_CONTROLLER_PKG} ${MOTOR_CONTROLLER_NODE} --ros-args -p serial_port:=${SERIAL_PORT} &
MOTOR_PID=$!
echo "Motor controller node started with PID: ${MOTOR_PID}"

echo "Nodes running in background. Press Ctrl+C to stop."
wait

echo "Jetson script finished."