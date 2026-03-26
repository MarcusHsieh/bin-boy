# Camera performance metrics for dashboard display

# Header with timestamp
std_msgs/Header header

# TensorRT inference time in milliseconds
float32 inference_time_ms

# Frames per second (FPS)
float32 fps

# Number of detections found by GPU
int32 gpu_detections

# Frame counter
uint64 frame_number
