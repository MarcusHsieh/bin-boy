#!/usr/bin/env python3
"""
Mock Person Detector for Simulation

Reads Gazebo model states to find person cylinders and projects them
to 2D bounding boxes using camera intrinsics. Publishes Detection2DArray
compatible with person_tracker.

This allows testing the person tracking and following pipeline in simulation
without requiring GPU-accelerated YOLOv5 inference.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D
import numpy as np
import math


class MockPersonDetector(Node):
    """
    Mock person detector that uses Gazebo ground truth to generate detections
    """

    def __init__(self):
        super().__init__('mock_person_detector')

        # Parameters
        self.declare_parameter('person_model_prefix', 'person')
        self.declare_parameter('person_height', 1.7)  # meters
        self.declare_parameter('person_radius', 0.3)  # meters
        self.declare_parameter('detection_confidence', 0.95)  # Simulated confidence
        self.declare_parameter('robot_name', 'bin_boy')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('debug_logging', False)

        # Get parameters
        self.person_prefix = self.get_parameter('person_model_prefix').value
        self.person_height = self.get_parameter('person_height').value
        self.person_radius = self.get_parameter('person_radius').value
        self.confidence = self.get_parameter('detection_confidence').value
        self.robot_name = self.get_parameter('robot_name').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.debug_logging = self.get_parameter('debug_logging').value

        # State
        self.camera_info = None
        self.robot_pose = None
        self.latest_model_states = None

        # Subscribers
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/person_detections',
            10
        )

        # Timer to publish detections at 10 Hz (matching real detector frequency)
        self.detection_timer = self.create_timer(0.1, self.publish_detections)

        self.get_logger().info('Mock Person Detector initialized')
        self.get_logger().info(f'Looking for models with prefix: "{self.person_prefix}"')
        self.get_logger().info(f'Robot name: {self.robot_name}')

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration"""
        self.camera_info = msg

    def model_states_callback(self, msg: ModelStates):
        """Store latest model states from Gazebo"""
        self.latest_model_states = msg

        # Extract robot pose for later transformation
        if self.robot_name in msg.name:
            robot_idx = msg.name.index(self.robot_name)
            self.robot_pose = msg.pose[robot_idx]

    def transform_world_to_camera(self, world_pos):
        """
        Transform world coordinates to camera coordinates

        Camera is mounted on the side of robot at 150mm height, facing forward.
        Camera frame (optical): X=right, Y=down, Z=forward (OpenCV convention)

        Args:
            world_pos: (x, y, z) in Gazebo world frame

        Returns:
            (x, y, z) in camera frame, or None if behind camera
        """
        if self.robot_pose is None:
            return None

        # Robot pose in world
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        rz = self.robot_pose.position.z

        # Robot orientation (yaw)
        # Convert quaternion to yaw
        qx = self.robot_pose.orientation.x
        qy = self.robot_pose.orientation.y
        qz = self.robot_pose.orientation.z
        qw = self.robot_pose.orientation.w

        # Yaw from quaternion
        robot_yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz))

        # Relative position in world frame
        dx = world_pos[0] - rx
        dy = world_pos[1] - ry
        dz = world_pos[2] - rz

        # Rotate to robot frame (base_link)
        # Robot frame: X=forward, Y=left, Z=up
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        x_robot = dx * cos_yaw + dy * sin_yaw
        y_robot = -dx * sin_yaw + dy * cos_yaw
        z_robot = dz

        # Camera offset from base_link (from URDF):
        # Camera is mounted on side at x=0.1575m (robot radius), y=0, z=0.15m
        # Camera points forward (same as robot X)
        camera_x_offset = 0.1575  # meters
        camera_y_offset = 0.0
        camera_z_offset = 0.15    # meters (150mm height from URDF)

        # Transform to camera frame (before optical frame rotation)
        x_cam_pre = x_robot - camera_x_offset
        y_cam_pre = y_robot - camera_y_offset
        z_cam_pre = z_robot - camera_z_offset

        # Now rotate to camera optical frame (OpenCV convention)
        # Camera frame: X=forward (was robot X)
        # Camera optical frame: X=right, Y=down, Z=forward
        # Rotation: X_optical = -Y_camera, Y_optical = -Z_camera, Z_optical = X_camera
        x_optical = -y_cam_pre  # Right
        y_optical = -z_cam_pre  # Down
        z_optical = x_cam_pre   # Forward

        # Check if point is in front of camera
        if z_optical <= 0.1:  # At least 10cm in front
            return None

        return (x_optical, y_optical, z_optical)

    def project_to_image(self, camera_pos):
        """
        Project 3D point in camera frame to 2D image coordinates

        Args:
            camera_pos: (x, y, z) in camera optical frame

        Returns:
            (u, v) pixel coordinates, or None if out of bounds
        """
        if self.camera_info is None:
            return None

        x, y, z = camera_pos

        # Camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Project to image plane
        u = (x * fx / z) + cx
        v = (y * fy / z) + cy

        # Check bounds
        if u < 0 or u >= self.camera_info.width or v < 0 or v >= self.camera_info.height:
            return None

        return (u, v)

    def create_bounding_box(self, center_pos):
        """
        Create bounding box for person cylinder

        Args:
            center_pos: (x, y, z) of person center in world frame

        Returns:
            (x1, y1, x2, y2) in pixels, or None if not visible
        """
        # Transform center to camera frame
        cam_center = self.transform_world_to_camera(center_pos)
        if cam_center is None:
            return None

        # Person cylinder: radius 0.3m, height 1.7m
        # We need to project the edges of the cylinder

        # Top and bottom of cylinder
        top_pos = (center_pos[0], center_pos[1], center_pos[2] + self.person_height / 2)
        bottom_pos = (center_pos[0], center_pos[1], center_pos[2] - self.person_height / 2)

        # Project top and bottom
        cam_top = self.transform_world_to_camera(top_pos)
        cam_bottom = self.transform_world_to_camera(bottom_pos)

        if cam_top is None or cam_bottom is None:
            return None

        pixel_top = self.project_to_image(cam_top)
        pixel_bottom = self.project_to_image(cam_bottom)
        pixel_center = self.project_to_image(cam_center)

        if pixel_top is None or pixel_bottom is None or pixel_center is None:
            return None

        # Estimate width from radius at center depth
        # Angular width = atan(radius / depth)
        depth = cam_center[2]
        angular_width = math.atan2(self.person_radius, depth)

        # Convert to pixels
        fx = self.camera_info.k[0]
        pixel_half_width = fx * math.tan(angular_width)

        # Bounding box
        x1 = max(0, pixel_center[0] - pixel_half_width)
        x2 = min(self.camera_info.width - 1, pixel_center[0] + pixel_half_width)
        y1 = max(0, pixel_top[1])
        y2 = min(self.camera_info.height - 1, pixel_bottom[1])

        # Ensure valid box
        if x2 <= x1 or y2 <= y1:
            return None

        return (x1, y1, x2, y2)

    def publish_detections(self):
        """
        Main detection loop - finds person models and publishes detections
        """
        if self.camera_info is None or self.latest_model_states is None:
            return

        detections_msg = Detection2DArray()
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = self.camera_frame

        # Find all person models
        person_count = 0
        for i, name in enumerate(self.latest_model_states.name):
            if self.person_prefix in name:
                person_count += 1
                pose = self.latest_model_states.pose[i]

                # Person position in world
                person_pos = (pose.position.x, pose.position.y, pose.position.z)

                # Create bounding box
                bbox = self.create_bounding_box(person_pos)

                if bbox is not None:
                    x1, y1, x2, y2 = bbox

                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = detections_msg.header

                    # Bounding box (center + size format)
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    size_x = x2 - x1
                    size_y = y2 - y1

                    detection.bbox.center.x = center_x
                    detection.bbox.center.y = center_y
                    detection.bbox.size_x = size_x
                    detection.bbox.size_y = size_y

                    # Hypothesis: person class with high confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.id = "person"
                    hypothesis.score = self.confidence
                    detection.results.append(hypothesis)

                    detections_msg.detections.append(detection)

                    if self.debug_logging:
                        self.get_logger().info(
                            f"Detected {name}: bbox=[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}], "
                            f"size={size_x:.0f}x{size_y:.0f}px"
                        )

        # Publish detections
        if detections_msg.detections:
            self.detection_pub.publish(detections_msg)

            if person_count > 0 and self.debug_logging:
                self.get_logger().info(
                    f"Published {len(detections_msg.detections)}/{person_count} person detections",
                    throttle_duration_sec=2.0
                )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MockPersonDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
