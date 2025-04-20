#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        # Subscribe to the raw image topic published by csi_camera_node
        # Using default reliable QoS for simplicity first
        self.subscription = self.create_subscription(
            Image,
            '/csi_camera_0/image_raw', # Assuming default sensor_id=0
            self.image_callback,
            10) # Default QoS depth
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Image viewer node started, subscribing to /csi_camera_0/image_raw")
        self.window_title = "ROS Image Viewer"
        cv2.namedWindow(self.window_title, cv2.WINDOW_AUTOSIZE)

    def image_callback(self, msg):
        # self.get_logger().debug('Received image') # Uncomment for verbose debugging
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # Display image
        cv2.imshow(self.window_title, cv_image)
        key = cv2.waitKey(1) & 0xFF # Need waitKey for imshow to work

        # Check if window was closed (using AUTOSIZE property as a proxy)
        # Or if 'q' or ESC is pressed
        if cv2.getWindowProperty(self.window_title, cv2.WND_PROP_AUTOSIZE) < 0 or \
           key == ord('q') or key == 27:
            self.get_logger().info("Closing image viewer window.")
            cv2.destroyWindow(self.window_title)
            # Optional: shutdown the node when window is closed
            # rclpy.shutdown()
            # Or just destroy the subscription/node components if you want the node to exit cleanly
            self.destroy_subscription(self.subscription)
            # Need a way to signal the main spin loop to exit, or handle node destruction better
            # For simplicity now, we just stop displaying. A real app might shut down.


def main(args=None):
    rclpy.init(args=args)
    image_viewer_node = ImageViewerNode()
    try:
        # Need spin to keep node alive and process callbacks
        rclpy.spin(image_viewer_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup OpenCV window just in case
        cv2.destroyAllWindows()
        # Destroy the node explicitly
        image_viewer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
