# Bin-Boy

ROS2 robotics project for autonomous navigation and perception.

## Build

> Run this once
```bash
colcon build
```

> Run this in every terminal
```bash
source install/setup.bash
```

## Launch

> Terminal 1 - Camera
```bash
ros2 launch csi_camera_cpp csi_camera_ipc.launch.py detection_frame_skip:=4 publish_annotated_image:=false
```

> Terminal 2 - 2D LIDAR
```bash
ros2 launch ldlidar_sl_ros2 ldlidar.launch.py
```

## Troubleshooting

> RTPS Error Debug
```bash
sudo rm -f /dev/shm/fastrtps*
```

> cap.read() error
```bash
sudo systemctl <status|restart> nvargus-daemon

gst-launch-1.0 nvarguscamerasrc ! fakesink
```