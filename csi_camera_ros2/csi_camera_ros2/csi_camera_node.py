#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage # Import CompressedImage
from cv_bridge import CvBridge, CvBridgeError # Import CvBridgeError
import time # Import time for potential debugging delays

# Pipeline function exactly from the working example script
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink max-buffers=1 sync=false" # Add appsink properties for low latency
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        # Declare parameters matching the pipeline function
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('capture_width', 1920)
        self.declare_parameter('capture_height', 1080)
        self.declare_parameter('display_width', 960)
        self.declare_parameter('display_height', 540)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('flip_method', 0)
        # Publish rate parameter - let's keep it matching framerate for now
        self.declare_parameter('publish_rate', 30.0)
        # Add parameters to control publishing - Defaulting to RAW only for lowest latency
        self.declare_parameter('publish_raw', True) # Default ON
        self.declare_parameter('publish_compressed', False) # Default OFF

        # Get parameters
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().integer_value
        self.capture_width = self.get_parameter('capture_width').get_parameter_value().integer_value
        self.capture_height = self.get_parameter('capture_height').get_parameter_value().integer_value
        self.display_width = self.get_parameter('display_width').get_parameter_value().integer_value
        self.display_height = self.get_parameter('display_height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_method = self.get_parameter('flip_method').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        # Get publishing control parameters
        self.publish_raw = self.get_parameter('publish_raw').get_parameter_value().bool_value
        self.publish_compressed = self.get_parameter('publish_compressed').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.image_pub_raw = None
        self.image_pub_compressed = None
        log_message_pubs = []

        # Conditionally create raw publisher
        if self.publish_raw:
            topic_name_raw = f'/csi_camera_{self.sensor_id}/image_raw'
            self.image_pub_raw = self.create_publisher(Image, topic_name_raw, 10) # Default QoS
            log_message_pubs.append(f"Raw: {topic_name_raw}")

        # Conditionally create compressed publisher
        if self.publish_compressed:
            topic_name_compressed = f'/csi_camera_{self.sensor_id}/image_raw/compressed' # Standard topic name
            self.image_pub_compressed = self.create_publisher(CompressedImage, topic_name_compressed, 10) # Default QoS
            log_message_pubs.append(f"Compressed: {topic_name_compressed}")

        if not log_message_pubs:
             self.get_logger().warn("Neither raw nor compressed publishing is enabled!")

        # Construct the pipeline
        self.pipeline = gstreamer_pipeline(
            sensor_id=self.sensor_id,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            display_width=self.display_width,
            display_height=self.display_height,
            framerate=self.framerate,
            flip_method=self.flip_method
        )
        self.get_logger().info(f"Using GStreamer pipeline: {self.pipeline}")

        # Attempt to open camera capture
        self.get_logger().info("Attempting to open camera...")
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        # Add a small delay and check if opened
        time.sleep(2.0) # Give pipeline time to initialize

        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Unable to open camera with GStreamer pipeline. Check pipeline string and camera connection.")
            # Attempt fallback with just sensor ID (less reliable)
            self.get_logger().info("Attempting fallback direct camera open...")
            self.cap = cv2.VideoCapture(self.sensor_id)
            if not self.cap.isOpened():
                 self.get_logger().error(f"Fallback direct open also failed. Shutting down.")
                 rclpy.shutdown()
                 return
            else:
                 self.get_logger().warn("Opened camera directly, not using GStreamer pipeline!")
        else:
             self.get_logger().info("Successfully opened camera using GStreamer pipeline.")


        # Create timer for publishing
        if publish_rate > 0:
             timer_period = 1.0 / publish_rate
             self.timer = self.create_timer(timer_period, self.timer_callback)
             if log_message_pubs:
                 self.get_logger().info(f"Publishing ({', '.join(log_message_pubs)}) at {publish_rate} Hz")
             else:
                 # If no publishers are active, no need for the timer? Or keep it for frame reading? Let's keep it for now.
                 self.get_logger().info(f"Timer active at {publish_rate} Hz, but no image publishers enabled.")
        else:
             self.get_logger().warn("Publish rate set to 0, images will not be published.")

        self.frame_count = 0

    def timer_callback(self):
        if not self.cap.isOpened():
             self.get_logger().warn("Timer callback called but camera is not open.")
             return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn(f"cap.read() returned False or empty frame.")
            return

        # Log success periodically
        # if self.frame_count % 60 == 0: # Log roughly every 2 seconds at 30fps
        self.get_logger().info(f"Successfully read frame {self.frame_count} (shape: {frame.shape}).")
        self.frame_count += 1

        # Prepare timestamp and frame_id once
        now = self.get_clock().now().to_msg()
        frame_id = f"camera_{self.sensor_id}_frame"

        # Conditionally publish raw image
        if self.publish_raw and self.image_pub_raw:
            try:
                image_msg_raw = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg_raw.header.stamp = now
                image_msg_raw.header.frame_id = frame_id
                self.image_pub_raw.publish(image_msg_raw)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge Error (Raw): {e}')


        # Conditionally publish compressed image
        if self.publish_compressed and self.image_pub_compressed:
            try:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = now
                compressed_msg.header.frame_id = frame_id
                compressed_msg.format = "jpeg"
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] # Quality 90
                compressed_msg.data = cv2.imencode('.jpg', frame, encode_param)[1].tobytes()
                self.image_pub_compressed.publish(compressed_msg)
            except Exception as e: # Catch potential encoding errors too
                 self.get_logger().error(f'Error compressing or publishing frame: {e}')

    def destroy_node(self):
        self.get_logger().info("Shutting down camera node.")
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera capture released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    csi_camera_node = CSICameraNode()
    if rclpy.ok(): # Check if node initialization failed
        try:
            rclpy.spin(csi_camera_node)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            csi_camera_node.get_logger().error(f"Unhandled exception in spin: {e}")
        finally:
            csi_camera_node.destroy_node()
            if rclpy.ok():
                 rclpy.shutdown()

if __name__ == '__main__':
    main()
