#!/usr/bin/env python3
"""
Person Tracking Node
Subscribes to YOLOv5 detections and publishes person locations
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
import math


class PersonTracker(Node):
    """
    Tracks detected persons and publishes their location
    Integrates with YOLOv5 detection from csi_camera_cpp
    """

    def __init__(self):
        super().__init__('person_tracker')

        # Declare parameters
        self.declare_parameter('detection_topic', '/camera/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('target_distance', 1.5)  # meters
        self.declare_parameter('max_tracking_distance', 5.0)  # meters
        self.declare_parameter('enable_following', False)

        # Get parameters
        detection_topic = self.get_parameter('detection_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.target_distance = self.get_parameter('target_distance').value
        self.max_distance = self.get_parameter('max_tracking_distance').value
        self.enable_following = self.get_parameter('enable_following').value

        # State
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_detection = None
        self.target_person = None

        # Create subscribers
        self.detection_sub = self.create_subscription(
            String,
            detection_topic,
            self.detection_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.person_location_pub = self.create_publisher(
            PointStamped,
            'person/location',
            10
        )

        self.tracking_status_pub = self.create_publisher(
            String,
            'person/tracking_status',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer for control loop (if following enabled)
        if self.enable_following:
            self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Person Tracker initialized')
        self.get_logger().info(f'Following mode: {self.enable_following}')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration info"""
        self.camera_info = msg

    def detection_callback(self, msg: String):
        """
        Process YOLO detections
        Expected format: JSON with detections array
        [{
            "class": "person",
            "confidence": 0.85,
            "bbox": [x1, y1, x2, y2]
        }, ...]
        """
        try:
            detections = json.loads(msg.data)

            # Filter for person detections
            persons = [
                d for d in detections
                if d.get('class') == 'person' and d.get('confidence', 0) >= self.min_confidence
            ]

            if not persons:
                self.latest_detection = None
                self.target_person = None
                self.publish_tracking_status('NO_PERSON')
                return

            # Select the person with highest confidence (or closest to center)
            # For now, use highest confidence
            self.target_person = max(persons, key=lambda p: p['confidence'])
            self.latest_detection = self.target_person

            # Estimate distance and angle
            distance, angle = self.estimate_person_pose(self.target_person)

            if distance is not None:
                # Publish person location
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'camera_link'
                point_msg.point.x = distance * math.cos(angle)
                point_msg.point.y = distance * math.sin(angle)
                point_msg.point.z = 0.0
                self.person_location_pub.publish(point_msg)

                # Publish tracking status
                status = f"TRACKING: dist={distance:.2f}m, angle={math.degrees(angle):.1f}deg, conf={self.target_person['confidence']:.2f}"
                self.publish_tracking_status(status)
            else:
                self.publish_tracking_status('PERSON_DETECTED_NO_DISTANCE')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')

    def estimate_person_pose(self, detection):
        """
        Estimate distance and angle to person from bounding box
        Returns (distance, angle) or (None, None) if cannot estimate

        Simple estimation based on:
        - Assume average person height ~1.7m
        - Use bbox height in image to estimate distance
        - Use bbox center x to estimate angle
        """
        if self.camera_info is None:
            return None, None

        bbox = detection['bbox']  # [x1, y1, x2, y2]
        x1, y1, x2, y2 = bbox

        # Bounding box center and dimensions
        bbox_center_x = (x1 + x2) / 2.0
        bbox_center_y = (y1 + y2) / 2.0
        bbox_height = y2 - y1
        bbox_width = x2 - x1

        # Camera parameters
        image_width = self.camera_info.width
        image_height = self.camera_info.height
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        cx = self.camera_info.k[2]  # Principal point x
        cy = self.camera_info.k[5]  # Principal point y

        # Estimate distance from bbox height
        # d = (real_height * fy) / bbox_height_pixels
        # Assume person is 1.7m tall, and bbox captures ~90% of person
        assumed_height = 1.7  # meters
        if bbox_height > 10:  # Sanity check
            distance = (assumed_height * fy) / bbox_height
        else:
            return None, None

        # Clamp distance to reasonable range
        distance = max(0.5, min(distance, self.max_distance))

        # Estimate angle from bbox center x
        # angle = atan((pixel_x - cx) / fx)
        pixel_offset = bbox_center_x - cx
        angle = math.atan2(pixel_offset, fx)

        return distance, angle

    def control_loop(self):
        """
        Generate velocity commands to follow person
        Simple proportional controller
        """
        if not self.enable_following:
            return

        if self.target_person is None:
            # No person detected, stop
            self.stop_robot()
            return

        # Estimate current person pose
        distance, angle = self.estimate_person_pose(self.target_person)

        if distance is None:
            self.stop_robot()
            return

        # Proportional control
        # Linear velocity: approach if too far, back off if too close
        distance_error = distance - self.target_distance
        linear_gain = 0.3
        linear_vel = linear_gain * distance_error

        # Clamp linear velocity
        max_linear_vel = 0.5  # m/s
        linear_vel = max(-max_linear_vel, min(linear_vel, max_linear_vel))

        # Angular velocity: turn to face person
        angular_gain = 1.0
        angular_vel = angular_gain * angle

        # Clamp angular velocity
        max_angular_vel = 1.0  # rad/s
        angular_vel = max(-max_angular_vel, min(angular_vel, max_angular_vel))

        # Dead zones
        if abs(distance_error) < 0.2:  # Within 20cm of target
            linear_vel = 0.0
        if abs(angle) < 0.1:  # Within ~6 degrees
            angular_vel = 0.0

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def publish_tracking_status(self, status: str):
        """Publish tracking status message"""
        msg = String()
        msg.data = status
        self.tracking_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PersonTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
