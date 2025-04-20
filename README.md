# Bin Boy
An autonomous trash can

## Install
```bash
sudo apt update && sudo apt install ros-foxy-cv-bridge ros-foxy-vision-msgs -y
sudo apt install libopencv-dev python3-opencv
```

## Camera (`csi_camera_cpp`)
> GStreamer -> /raw_image (unique_ptr)  
> ROS msg --cv_bridge--> OpenCV compatible viewing  
> Minimize latency with unique_ptr (no copying w/n nodes in same process)  
> Person detector subscribes to /image_raw running pre-trained MobileNet-SSD Caffe model performing inference
```bash
source install/setup.bash

# defaults: detector ON, skip=1, annotated image OFF

ros2 launch csi_camera_cpp csi_camera_ipc.launch.py

ros2 launch csi_camera_cpp csi_camera_ipc.launch.py run_detector:=false

ros2 launch csi_camera_cpp csi_camera_ipc.launch.py detection_frame_skip:=4 publish_annotated_image:=false
```

## Debug
```bash
sudo systemctl <status|restart> nvargus-daemon

gst-launch-1.0 nvarguscamerasrc ! fakesink
```