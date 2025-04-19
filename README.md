# Bin Boy
An autonomous trash can 

## Calibrate
> 10x8 checkerboard 15mm square
```bash
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 image:=/csi_camera_0/image_raw camera:=/csi_camera_0
```

## Run

```bash
ros2 run csi_camera_ros2 csi_camera_node
```