# Bin-Boy

ROS2 autonomous trash can for person following and navigation.

## Features

- **Person Tracking**: YOLOv5 TensorRT detection with color-based re-identification
- **Wide-Angle Vision**: 200° FOV camera with barrel distortion correction
- **360° LIDAR**: LD14P for mapping and obstacle avoidance
- **Kiwi Drive**: Omnidirectional 3-wheel platform
- **Sensor Fusion**: IMU + wheel odometry with EKF

## Hardware

- NVIDIA Jetson Nano 4GB
- Waveshare IMX219-200 CSI Camera (8MP, 200° FOV)
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

> Camera with Person Detection (CSI - Recommended)
```bash
# Calibrated for Waveshare IMX219-200 with barrel correction
ros2 launch camera_cpp camera_ipc.launch.py camera_type:=csi
```

> Camera with Person Detection (USB)
```bash
ros2 launch camera_cpp camera_ipc.launch.py camera_type:=usb device_id:=0
```

> 2D LIDAR
```bash
ros2 launch ldlidar_sl_ros2 ld14p.launch.py
```

> Full Control System (Motors + IMU + EKF)
```bash
ros2 launch bin_boy_control full_control.launch.py
```

## Camera Calibration

The CSI camera comes pre-calibrated for the Waveshare IMX219-200 lens. To recalibrate:

```bash
# 1. Start camera without correction
ros2 launch camera_cpp camera_ipc.launch.py camera_type:=csi enable_distortion_correction:=false

# 2. Run interactive calibration tool
ros2 run camera_cpp calibrate_distortion

# 3. Adjust sliders until straight lines appear straight
# 4. Press 's' to save, 'q' to quit and get launch parameters
```

## Camera Features

- **BGR color output** for person re-identification
- **Barrel distortion correction** (k1=-0.130, calibrated)
- **Auto white balance** enabled by default
- **TensorRT GPU acceleration** (~15ms inference @ 15fps)
- **Person detection** with bounding boxes
- **Zero-copy IPC** between camera and detector nodes

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
