#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def gstreamer_pipeline(sensor_id=0, sensor_mode=0, capture_width=1920, capture_height=1080, framerate=20):
    """Constructs the GStreamer pipeline string."""
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! "
        f"video/x-raw(memory:NVMM),width={capture_width},height={capture_height},framerate={framerate}/1,format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true"  # Added drop=true for potential performance improvement
    )
    return pipeline

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        # Declare parameters
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('sensor_mode', 0)
        self.declare_parameter('capture_width', 1920)
        self.declare_parameter('capture_height', 1080)
        self.declare_parameter('framerate', 20)
        self.declare_parameter('publish_rate', 20.0) # Hz

        # Get parameters
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().integer_value
        self.sensor_mode = self.get_parameter('sensor_mode').get_parameter_value().integer_value
        self.capture_width = self.get_parameter('capture_width').get_parameter_value().integer_value
        self.capture_height = self.get_parameter('capture_height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        topic_name_raw = f'/csi_camera_{self.sensor_id}/image_raw'
        topic_name_compressed = f'{topic_name_raw}/compressed'
        self.image_pub_raw = self.create_publisher(Image, topic_name_raw, 10)
        self.image_pub_compressed = self.create_publisher(CompressedImage, topic_name_compressed, 10)
        self.bridge = CvBridge()

        self.pipeline = gstreamer_pipeline(
            sensor_id=self.sensor_id,
            sensor_mode=self.sensor_mode,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            framerate=self.framerate
        )
        self.get_logger().info(f"Using GStreamer pipeline: {self.pipeline}")

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Unable to open camera with sensor ID {self.sensor_id}")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(f"Started camera node for sensor ID {self.sensor_id}. Raw: {topic_name_raw}, Compressed: {topic_name_compressed}")

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

        # Publish compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = now
        compressed_msg.header.frame_id = frame_id
        compressed_msg.format = "jpeg"
        compressed_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
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
