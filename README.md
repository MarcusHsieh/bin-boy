# Bin-Boy

ROS2 autonomous trash can for person following and navigation.

## Features

- **Person Tracking**: YOLOv5 TensorRT detection with spatial-based re-identification
- **Wide-Angle Vision**: 148° FOV mono camera with global shutter
- **360° LIDAR**: LD14P for mapping and obstacle avoidance
- **Kiwi Drive**: Omnidirectional 3-wheel platform
- **Sensor Fusion**: IMU + wheel odometry with EKF

## Hardware

- NVIDIA Jetson Nano 4GB
- OV9281 USB Camera (1MP, 148° FOV, Global Shutter, Mono)
- LD14P 2D LIDAR
- 3x Feetech STS3215 Servos (kiwi drive)
- MPU-6050 IMU

## Build

> Run this once
```bash
colcon build
```

> Run this in every terminal
```bash
source install/setup.bash
```

## Quick Start

> Camera with Person Detection (CSI)
```bash
# Calibrated for Waveshare IMX219-200 with barrel correction
ros2 launch camera_cpp camera_ipc.launch.py camera_type:=csi
```

> Person Tracking (USB Camera)
```bash
# Complete tracking pipeline: camera + detection + spatial tracking
ros2 launch bin_boy_perception person_tracking.launch.py
```

> Person Tracking (CSI Camera with Color Re-ID)
```bash
# Enable color-based re-identification for CSI color camera
ros2 launch bin_boy_perception person_tracking.launch.py camera_type:=csi enable_color_tracking:=true enable_distortion_correction:=true
```

> Person Tracking with Robot Following
```bash
# Enable autonomous following behavior (requires Nav2)
ros2 launch bin_boy_perception person_tracking.launch.py enable_following:=true target_distance:=0.8
```

> 2D LIDAR (raw)
```bash
ros2 launch ldlidar_sl_ros2 ld14p.launch.py
```

> 2D LIDAR (filtered scan ignoring points <=150mm from base_laser)
```bash
ros2 launch ldlidar_sl_ros2 ld14p_filtered.launch.py
```

> Full Control System (Motors + IMU + EKF)
```bash
ros2 launch bin_boy_control full_control.launch.py
```

## Camera Calibration

**OV9281 USB Camera**: No calibration needed (minimal distortion at 148° FOV).

**CSI Camera**: Pre-calibrated for Waveshare IMX219-200 lens. To recalibrate:

```bash
# 1. Start camera without correction
ros2 launch camera_cpp camera_ipc.launch.py camera_type:=csi enable_distortion_correction:=false

# 2. Run interactive calibration tool
ros2 run camera_cpp calibrate_distortion

# 3. Adjust sliders until straight lines appear straight
# 4. Press 's' to save, 'q' to quit and get launch parameters
```

## Camera Features

- **Grayscale mono output** (OV9281 sensor)
- **Global shutter** for motion artifact-free captures
- **No distortion correction** needed (148° FOV, minimal distortion)
- **TensorRT GPU acceleration** (~15ms inference @ 15fps)
- **Person detection** with bounding boxes
- **Zero-copy IPC** between camera and detector nodes
- **Spatial tracking** with proximity-based re-identification

## Troubleshooting

> RTPS Error Debug
```bash
sudo rm -f /dev/shm/fastrtps*
```

> cap.read() error
```bash
# cap.read() error
sudo systemctl restart nvargus-daemon

sudo systemctl status nvargus-daemon

gst-launch-1.0 nvarguscamerasrc ! fakesink
```
