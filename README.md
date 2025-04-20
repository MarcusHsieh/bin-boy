# Bin Boy
An autonomous trash can

## Install
```bash
sudo apt update && sudo apt install ros-foxy-cv-bridge -y
```

## Camera (`csi_camera_cpp`)
> GStreamer -> /raw_image (unique_ptr)  
> ROS msg --cv_bridge--> OpenCV compatible viewing  
> Minimize latency with unique_ptr (no copying w/n nodes in same process)
```bash
ros2 launch csi_camera_cpp csi_camera_ipc.launch.py
```
Launch camera publisher + image viewer
`CSICameraNode` accepts parameters (launch file or cmd line)
*   `sensor_id`: Camera sensor ID (default: 0)
*   `capture_width`, `capture_height`: Resolution requested from `nvarguscamerasrc`
*   `display_width`, `display_height`: Resolution set after `nvvidconv` (used for CameraInfo and potentially downscaling)
*   `framerate`: Requested framerate
*   `flip_method`: Image flip/rotation (0: none, 1: CCW 90, 2: 180, 3: CW 90, 4: HFlip, 5: VFlip+HFlip, 6: VFlip, 7: HFlip+CW 90)
*   `publish_rate`: Rate at which node attempts to read and publish frames
*   `frame_id`: ROS frame ID for published messages

## Debug
```bash
sudo systemctl <restart|status> nvargus-daemon
