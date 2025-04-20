#!/bin/bash

IMAGE_NAME="bin_boy_laptop"
TAG="foxy-qt-joystick"

if [[ "$(docker images -q ${IMAGE_NAME}:${TAG} 2> /dev/null)" == "" ]]; then
  echo "Error: Docker image ${IMAGE_NAME}:${TAG} not found."
  echo "Please build the image first using ./build_laptop_image.sh"
  exit 1
fi

xhost +local:docker

HOST_ROS2_WORKSPACE="$(pwd)"
CONTAINER_ROS2_WORKSPACE="/ros2_ws"

echo "Running Docker container: ${IMAGE_NAME}:${TAG}..."
echo "Mounting host workspace: ${HOST_ROS2_WORKSPACE} -> ${CONTAINER_ROS2_WORKSPACE}"

docker run -it --rm \
    --hostname ros2-laptop-container \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --runtime nvidia
    --net=host
    --ipc=host \
    --privileged  \
    -v "${HOST_ROS2_WORKSPACE}:${CONTAINER_ROS2_WORKSPACE}" \
    "${IMAGE_NAME}:${TAG}" \
    /bin/bash -c " \
        echo '--- Inside Container ---'; \
        echo 'Sourcing ROS setup...'; \
        source /opt/ros/foxy/setup.bash; \
        echo 'Changing to workspace: ${CONTAINER_ROS2_WORKSPACE}'; \
        cd ${CONTAINER_ROS2_WORKSPACE}; \
        echo 'Attempting to build workspace (if needed)...'; \
        # Optional: Build the workspace inside the container if src was mounted
        # Consider running this manually first if needed: colcon build --symlink-install; \
        echo 'Sourcing local workspace...'; \
        if [ -f install/setup.bash ]; then source install/setup.bash; else echo 'Local setup.bash not found. Build the workspace first inside the container (colcon build).'; fi; \
        echo '------------------------'; \
        echo 'Starting bash shell. Run your ROS commands, e.g.:'; \
        echo '  ros2 run motor_controller qt_joystick'; \
        echo '------------------------'; \
        exec /bin/bash"

echo "Docker container exited."