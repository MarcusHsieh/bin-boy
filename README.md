# Bin Boy
An autonomous trash can 

## Install
```bash
sudo apt update && sudo apt install ros-foxy-cv-bridge -y
sudo apt update && sudo apt install ros-foxy-image-transport-plugins -y
```

## Calibrate
> 10x8 checkerboard 15mm square  
> image_raw/compressed
```bash
ros2 bag record /csi_camera_0/image_raw -o calibration_video
```
```bash
ros2 run camera_calibration cameracalibrator --ros-args --remap image:=/csi_camera_0/image_raw --remap camera:=/csi_camera_0 -p size:=7x9 -p square:=0.015
```
```bash
ros2 bag play calibration_video
```

## Run

```bash
ros2 run csi_camera_ros2 csi_camera_node
```