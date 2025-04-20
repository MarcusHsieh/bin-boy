#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# Revert pipeline EXACTLY to the working example script's structure and formatting
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640, # Keep VGA default
    capture_height=480, # Keep VGA default
    display_width=640, # Match capture for ROS node
    display_height=480, # Match capture for ROS node
    framerate=20, # Keep 20fps default
    flip_method=0, # Keep flip_method=0
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink" # Removed drop=true
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width, # Use display_width here
            display_height, # Use display_height here
        )
    )

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        # Declare parameters
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('sensor_mode', 0) # Using mode 0
        self.declare_parameter('capture_width', 640)  # Default VGA
        self.declare_parameter('capture_height', 480) # Default VGA
        # Add display width/height params used in the pipeline function now
        self.declare_parameter('display_width', 640)
        self.declare_parameter('display_height', 480)
        self.declare_parameter('framerate', 20)       # Default 20fps
        self.declare_parameter('flip_method', 0)      # Default flip_method
        self.declare_parameter('publish_rate', 20.0)  # Match framerate
        self.declare_parameter('publish_compressed', True) # Publish compressed by default

        # Get parameters
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().integer_value
        self.sensor_mode = self.get_parameter('sensor_mode').get_parameter_value().integer_value
        self.capture_width = self.get_parameter('capture_width').get_parameter_value().integer_value
        self.capture_height = self.get_parameter('capture_height').get_parameter_value().integer_value
        # Get display width/height params
        self.display_width = self.get_parameter('display_width').get_parameter_value().integer_value
        self.display_height = self.get_parameter('display_height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_method = self.get_parameter('flip_method').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_compressed = self.get_parameter('publish_compressed').get_parameter_value().bool_value

        # Define a QoS profile: Best Effort, Depth 1
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        topic_name_raw = f'/csi_camera_{self.sensor_id}/image_raw'
        self.image_pub_raw = self.create_publisher(Image, topic_name_raw, qos_profile) # Use QoS profile
        self.image_pub_compressed = None
        log_message = f"Started camera node for sensor ID {self.sensor_id}. Publishing Raw: {topic_name_raw}"
        if self.publish_compressed:
            topic_name_compressed = f'{topic_name_raw}/compressed'
            self.image_pub_compressed = self.create_publisher(CompressedImage, topic_name_compressed, qos_profile) # Use QoS profile
            log_message += f", Compressed: {topic_name_compressed}"
        self.bridge = CvBridge()

        self.pipeline = gstreamer_pipeline(
            # Pass all params to the pipeline function now
            sensor_id=self.sensor_id,
            sensor_mode=self.sensor_mode,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            display_width=self.display_width,
            display_height=self.display_height,
            framerate=self.framerate,
            flip_method=self.flip_method
        )
        self.get_logger().info(f"Using GStreamer pipeline: {self.pipeline}") # Log the exact pipeline being used

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Unable to open camera with sensor ID {self.sensor_id}")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(log_message) # Use the constructed log message

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f"Error: Unable to read frame from camera with sensor ID {self.sensor_id}")
            # Consider adding logic to attempt reconnection or shutdown
            return

        # Prepare timestamp and frame_id
        now = self.get_clock().now().to_msg()
        frame_id = f"camera_{self.sensor_id}_frame"

        # Publish raw image
        image_msg_raw = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        image_msg_raw.header.stamp = now
        image_msg_raw.header.frame_id = frame_id
        self.image_pub_raw.publish(image_msg_raw)

        # Publish compressed image only if enabled
        if self.publish_compressed and self.image_pub_compressed:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = now
            compressed_msg.header.frame_id = frame_id
            compressed_msg.format = "jpeg"
            # Use imencode parameters for potential speedup (lower quality)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] # Quality 90 (0-100)
            compressed_msg.data = cv2.imencode('.jpg', frame, encode_param)[1].tobytes()
            self.image_pub_compressed.publish(compressed_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down camera node.")
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    csi_camera_node = CSICameraNode()
    try:
        rclpy.spin(csi_camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        csi_camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
