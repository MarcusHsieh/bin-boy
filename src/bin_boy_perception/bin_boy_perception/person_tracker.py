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
from vision_msgs.msg import Detection2DArray
from bin_boy_interfaces.msg import PersonTracking, PersonTrackingArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import message_filters
from enum import Enum
from concurrent.futures import ThreadPoolExecutor
from .color_histogram import ColorHistogram


class TrackingState(Enum):
    """Person tracking state machine states"""
    IDLE = 0          # No target selected, waiting
    TRACKING = 1      # Actively tracking target person
    LOST = 2          # Target not detected in current frame
    REACQUIRING = 3   # Searching for lost target using color
    LOCKED = 4        # User-confirmed target lock (highest priority)


class PersonTracker(Node):
    """
    Tracks detected persons and publishes their location
    Integrates with YOLOv5 detection from csi_camera_cpp
    """

    def __init__(self):
        super().__init__('person_tracker')

        # Declare parameters
        self.declare_parameter('detection_topic', '/person_detections')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('target_distance', 1.5)  # meters
        self.declare_parameter('max_tracking_distance', 5.0)  # meters
        self.declare_parameter('enable_following', False)
        self.declare_parameter('enable_color_tracking', True)
        self.declare_parameter('lost_timeout', 3.0)  # seconds before giving up on lost target
        self.declare_parameter('reacquire_color_threshold', 0.6)  # minimum color similarity for re-acquisition
        self.declare_parameter('reid_color_threshold', 0.7)  # minimum color similarity for person re-identification
        self.declare_parameter('adaptive_histogram_alpha', 0.15)  # histogram update rate (0=no adaptation, 1=full replacement)
        self.declare_parameter('id_memory_timeout', 30.0)  # seconds to remember person IDs (for re-identification)
        self.declare_parameter('debug_logging', False)  # enable verbose debug logging

        # Get parameters
        detection_topic = self.get_parameter('detection_topic').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.target_distance = self.get_parameter('target_distance').value
        self.max_distance = self.get_parameter('max_tracking_distance').value
        self.enable_following = self.get_parameter('enable_following').value
        self.enable_color_tracking = self.get_parameter('enable_color_tracking').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.reacquire_threshold = self.get_parameter('reacquire_color_threshold').value
        self.reid_threshold = self.get_parameter('reid_color_threshold').value
        self.adaptive_alpha = self.get_parameter('adaptive_histogram_alpha').value
        self.debug_logging = self.get_parameter('debug_logging').value

        # State
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_detection = None
        self.target_person = None
        self.target_color_histogram = None  # HSV histogram of target person
        self.current_image = None  # Current BGR image for color extraction

        # Tracking state machine
        self.tracking_state = TrackingState.IDLE
        self.last_seen_time = None  # Last time target was detected
        self.frames_lost = 0  # Number of consecutive frames without detection
        self.last_message_time = None  # Last time ANY detection message was received (for watchdog)

        # Person ID tracking
        self.next_person_id = 1  # Next ID to assign
        self.person_id_map = {}  # {person_id: {'bbox', 'histogram', 'last_seen', 'frames_missing'}}
        self.target_person_id = None  # ID of locked target
        self.id_timeout = self.get_parameter('id_memory_timeout').value  # seconds before removing lost ID
        self.id_max_distance = 150.0  # pixels - max movement between frames for same ID

        # Color histogram extractor for person re-identification
        # Optimized: 16×16×8 bins (2,048 total) for 5x speedup
        self.color_hist = ColorHistogram(h_bins=16, s_bins=16, v_bins=8)

        # Thread pool for parallel person processing
        self.thread_pool = ThreadPoolExecutor(max_workers=4)

        # Create synchronized subscribers for detections + images
        if self.enable_color_tracking:
            # Use message_filters for time synchronization
            self.detection_sub = message_filters.Subscriber(self, Detection2DArray, detection_topic)
            self.image_sub = message_filters.Subscriber(self, Image, image_topic)

            # Synchronize detection and image messages (100ms tolerance)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.detection_sub, self.image_sub],
                queue_size=10,
                slop=0.1  # 100ms tolerance
            )
            self.sync.registerCallback(self.synchronized_callback)

            self.get_logger().info('Color tracking enabled - using synchronized detection + image')
        else:
            # Fallback: detections only (no color tracking)
            self.detection_sub = self.create_subscription(
                Detection2DArray,
                detection_topic,
                self.detection_only_callback,
                10
            )
            self.get_logger().info('Color tracking disabled - detection only mode')

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

        # Publish target person detection separately (for visualization)
        self.target_detection_pub = self.create_publisher(
            Detection2DArray,
            'person/target_detection',
            10
        )

        # Publish annotated image with target highlighted
        self.annotated_image_pub = self.create_publisher(
            Image,
            'person/tracking_image',
            10
        )

        # Publish structured person tracking data (for RQT dashboard)
        self.tracking_data_pub = self.create_publisher(
            PersonTrackingArray,
            'person/tracking_data',
            10
        )

        # Timer for control loop (if following enabled)
        if self.enable_following:
            self.control_timer = self.create_timer(0.1, self.control_loop)

        # Watchdog timer to detect when detection messages stop arriving
        # This handles the case where camera node stops publishing when detections=0
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)  # Check every 500ms

        self.get_logger().info('Person Tracker initialized')
        self.get_logger().info(f'Following mode: {self.enable_following}')

    def watchdog_check(self):
        """
        Watchdog timer to handle state transitions when detection messages stop arriving.

        This fixes the bug where camera node stops publishing when detections=0,
        causing the tracker to freeze in TRACKING state.
        """
        # If we've never received a message, nothing to check
        if self.last_message_time is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_message = current_time - self.last_message_time

        # If messages are arriving normally (within 1 second), do nothing
        if time_since_message < 1.0:
            return

        # Messages have stopped - handle state transitions
        if self.tracking_state == TrackingState.TRACKING:
            # We were tracking but messages stopped = target lost
            self.frames_lost += 1
            self.tracking_state = TrackingState.LOST
            self.target_person = None
            self.get_logger().warn(f'WATCHDOG: No messages for {time_since_message:.1f}s - LOST')
            self.publish_tracking_status(f'LOST: No detections ({self.frames_lost} frames)')

            # Publish empty tracking data
            self.publish_tracking_data([], None)

        elif self.tracking_state == TrackingState.LOST:
            # Still lost, transition to REACQUIRING
            self.frames_lost += 1
            self.tracking_state = TrackingState.REACQUIRING
            self.get_logger().warn(f'WATCHDOG: Still no messages - REACQUIRING ({self.frames_lost} frames)')
            self.publish_tracking_status(f'REACQUIRING: Lost {self.frames_lost} frames')

            # Publish empty tracking data
            self.publish_tracking_data([], None)

        elif self.tracking_state == TrackingState.REACQUIRING:
            # Check if we should give up entirely
            if self.last_seen_time is not None:
                time_lost = current_time - (self.last_seen_time / 1e9)
                if time_lost > self.lost_timeout:
                    # Timeout: reset to IDLE
                    self.tracking_state = TrackingState.IDLE
                    self.target_color_histogram = None
                    self.target_person_id = None
                    self.last_seen_time = None
                    self.frames_lost = 0
                    self.get_logger().warn(f'WATCHDOG: Lost target for {time_lost:.1f}s - IDLE')
                    self.publish_tracking_status(f'IDLE: Lost target after {time_lost:.1f}s')

                    # Publish empty tracking data
                    self.publish_tracking_data([], None)

    def camera_info_callback(self, msg: CameraInfo):
        """Store camera calibration info"""
        self.camera_info = msg

    def assign_person_ids(self, persons):
        """
        Assign persistent IDs to detected persons using spatial and histogram matching

        Process:
        1. Try spatial matching first (proximity < 150px)
        2. If no spatial match, try histogram-based re-identification (similarity > 0.7)
        3. If still no match, create new ID

        Args:
            persons: List of person dicts with 'bbox', 'confidence', 'histogram', etc.

        Returns:
            Updated persons list with 'id' field added
        """
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Clean up old IDs that haven't been seen recently
        ids_to_remove = []
        for person_id, info in self.person_id_map.items():
            time_missing = current_time - info['last_seen']
            if time_missing > self.id_timeout:
                ids_to_remove.append(person_id)

        for person_id in ids_to_remove:
            del self.person_id_map[person_id]
            if self.debug_logging:
                self.get_logger().info(f"Removed stale ID#{person_id} (not seen for {self.id_timeout}s)")

        # Assign IDs to current detections
        assigned_ids = set()

        for person in persons:
            bbox = person['bbox']
            center_x = (bbox[0] + bbox[2]) / 2.0
            center_y = (bbox[1] + bbox[3]) / 2.0

            # Try to match with existing IDs based on proximity
            best_id = None
            best_distance = self.id_max_distance

            for person_id, info in self.person_id_map.items():
                if person_id in assigned_ids:
                    continue

                prev_bbox = info['bbox']
                prev_center_x = (prev_bbox[0] + prev_bbox[2]) / 2.0
                prev_center_y = (prev_bbox[1] + prev_bbox[3]) / 2.0

                distance = math.sqrt((center_x - prev_center_x)**2 + (center_y - prev_center_y)**2)

                if distance < best_distance:
                    best_distance = distance
                    best_id = person_id

            # If no spatial match, try histogram-based re-identification
            if best_id is None and self.enable_color_tracking:
                person_hist = person.get('histogram')
                if person_hist is not None:
                    best_hist_similarity = 0.0
                    best_hist_id = None

                    # Compare against all stored histograms
                    for person_id, info in self.person_id_map.items():
                        if person_id in assigned_ids:
                            continue

                        stored_hist = info.get('histogram')
                        if stored_hist is not None:
                            similarity = self.color_hist.compare_histograms(person_hist, stored_hist)
                            if similarity > best_hist_similarity:
                                best_hist_similarity = similarity
                                best_hist_id = person_id

                    # If good histogram match, reuse that ID (person re-identification)
                    if best_hist_similarity >= self.reid_threshold:
                        best_id = best_hist_id
                        if self.debug_logging:
                            self.get_logger().info(
                                f"Re-identified person as ID#{best_id} (color similarity: {best_hist_similarity:.2f})"
                            )

            # Assign ID
            if best_id is not None:
                # Matched existing person (either spatial or histogram)
                person['id'] = best_id
                assigned_ids.add(best_id)
                self.person_id_map[best_id]['bbox'] = bbox
                self.person_id_map[best_id]['last_seen'] = current_time
                self.person_id_map[best_id]['frames_missing'] = 0
                # Update histogram if available
                if 'histogram' in person and person['histogram'] is not None:
                    self.person_id_map[best_id]['histogram'] = person['histogram']
            else:
                # New person - assign new ID
                person['id'] = self.next_person_id
                # Store histogram if available
                person_hist = person.get('histogram')
                self.person_id_map[self.next_person_id] = {
                    'bbox': bbox,
                    'histogram': person_hist,
                    'last_seen': current_time,
                    'frames_missing': 0
                }
                assigned_ids.add(self.next_person_id)
                if self.debug_logging:
                    self.get_logger().info(f"New person detected: ID#{self.next_person_id}")
                self.next_person_id += 1

        # Mark missing IDs
        for person_id in self.person_id_map:
            if person_id not in assigned_ids:
                self.person_id_map[person_id]['frames_missing'] += 1

        return persons

    def extract_histograms_parallel(self, persons):
        """
        Extract histograms for multiple persons in parallel using ThreadPoolExecutor

        Args:
            persons: List of person dicts with 'bbox'

        Returns:
            Updated persons list with 'histogram' field added
        """
        if not persons or self.current_image is None:
            return persons

        # Extract histograms in parallel
        def extract_for_person(person):
            hist = self.color_hist.extract_histogram(self.current_image, person['bbox'])
            return (person, hist)

        # Submit all extraction tasks
        futures = [self.thread_pool.submit(extract_for_person, p) for p in persons]

        # Collect results
        for future in futures:
            person, hist = future.result()
            person['histogram'] = hist

            # Note: ID assignment happens after histogram extraction,
            # so we can't store in person_id_map here. Will be done after assign_person_ids().

        return persons

    def synchronized_callback(self, detections_msg: Detection2DArray, image_msg: Image):
        """
        Synchronized callback for detections + images (for color tracking)
        """
        try:
            # Convert ROS Image to OpenCV BGR
            self.current_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            self.current_image = None

        # Process detections with color information available
        self.process_detections(detections_msg)

    def detection_only_callback(self, msg: Detection2DArray):
        """
        Fallback callback for detections only (no color tracking)
        """
        self.current_image = None
        self.process_detections(msg)

    def process_detections(self, msg: Detection2DArray):
        """
        Process YOLO detections from vision_msgs/Detection2DArray
        Format:
        - detections[].bbox (center x/y, size_x/y)
        - detections[].results[].hypothesis.id = "person"
        - detections[].results[].hypothesis.score = confidence
        """
        # Update message watchdog timestamp
        self.last_message_time = self.get_clock().now().nanoseconds / 1e9

        try:
            # Convert Detection2DArray to internal format
            persons = []
            for detection in msg.detections:
                # Check if this is a person detection
                if not detection.results:
                    continue

                # Get the best hypothesis (highest score)
                best_result = max(detection.results, key=lambda r: r.score)

                if best_result.id != "person":
                    continue

                confidence = best_result.score
                if confidence < self.min_confidence:
                    continue

                # Convert bbox from (center, size) to (x1, y1, x2, y2)
                # Pose2D has x, y directly (not position.x)
                center_x = detection.bbox.center.x
                center_y = detection.bbox.center.y
                size_x = detection.bbox.size_x
                size_y = detection.bbox.size_y

                x1 = center_x - size_x / 2.0
                y1 = center_y - size_y / 2.0
                x2 = center_x + size_x / 2.0
                y2 = center_y + size_y / 2.0

                persons.append({
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'center': (center_x, center_y),
                    'size': (size_x, size_y)
                })

            # Extract histograms in parallel for all persons (BEFORE ID assignment)
            # This allows us to use histogram matching for re-identification
            if self.enable_color_tracking and self.current_image is not None:
                persons = self.extract_histograms_parallel(persons)

            # Assign persistent IDs to all detected persons (uses histogram matching)
            persons = self.assign_person_ids(persons)

            # State machine: handle no persons detected
            if not persons:
                self.frames_lost += 1
                self.target_person = None

                # Check if we should give up
                if self.last_seen_time is not None:
                    time_lost = (self.get_clock().now().nanoseconds - self.last_seen_time) / 1e9
                    if time_lost > self.lost_timeout:
                        # Timeout: reset to IDLE
                        self.tracking_state = TrackingState.IDLE
                        self.target_color_histogram = None
                        self.last_seen_time = None
                        self.get_logger().warn(f"Target lost for {time_lost:.1f}s - resetting to IDLE")
                        self.publish_tracking_status(f'IDLE: Lost target after {time_lost:.1f}s')
                    elif self.tracking_state == TrackingState.TRACKING:
                        # Just lost, transition to LOST
                        self.tracking_state = TrackingState.LOST
                        self.publish_tracking_status(f'LOST: {self.frames_lost} frames')
                    elif self.tracking_state == TrackingState.LOST:
                        # Still lost, transition to REACQUIRING
                        self.tracking_state = TrackingState.REACQUIRING
                        self.publish_tracking_status(f'REACQUIRING: Lost {self.frames_lost} frames')
                else:
                    # Never had a target
                    self.tracking_state = TrackingState.IDLE
                    self.publish_tracking_status('IDLE: No person detected')

                # Still publish tracking data updates even with empty persons list
                # This prevents the dashboard from "freezing" when nobody is in frame
                if self.current_image is not None:
                    self.publish_tracking_image(msg.detections, persons, None)
                self.publish_tracking_data(persons, None)

                return

            # State machine: persons detected - select target
            self.frames_lost = 0
            self.last_seen_time = self.get_clock().now().nanoseconds

            if self.tracking_state == TrackingState.IDLE:
                # First detection: select highest confidence person and capture histogram
                if self.enable_color_tracking and self.current_image is not None:
                    self.target_person = max(persons, key=lambda p: p['confidence'])
                    self.target_person_id = self.target_person.get('id')
                    self.target_color_histogram = self.target_person.get('histogram')
                    self.tracking_state = TrackingState.TRACKING

                    # Log histogram capture
                    if self.target_color_histogram is not None:
                        self.get_logger().info(
                            f"TARGET LOCKED: ID#{self.target_person_id} (confidence: {self.target_person['confidence']:.2f})"
                        )
                        if self.debug_logging:
                            hist_sum = np.sum(self.target_color_histogram)
                            hist_shape = self.target_color_histogram.shape
                            self.get_logger().info(
                                f"  Histogram: {hist_shape} bins, sum={hist_sum:.2f} | "
                                f"BBox: [{self.target_person['bbox'][0]:.0f}, {self.target_person['bbox'][1]:.0f}, "
                                f"{self.target_person['bbox'][2]:.0f}, {self.target_person['bbox'][3]:.0f}]"
                            )
                else:
                    # No color tracking, just track highest confidence
                    self.target_person = max(persons, key=lambda p: p['confidence'])
                    self.target_person_id = self.target_person.get('id')
                    self.tracking_state = TrackingState.TRACKING

            elif self.tracking_state in [TrackingState.TRACKING, TrackingState.REACQUIRING]:
                # Subsequent detections: match using color histogram
                if self.enable_color_tracking and self.current_image is not None and self.target_color_histogram is not None:
                    self.target_person = self.color_hist.match_person(
                        self.current_image,
                        persons,
                        self.target_color_histogram,
                        confidence_weight=0.4,
                        color_weight=0.6
                    )

                    if self.target_person:
                        color_sim = self.target_person.get('color_similarity', 0.0)
                        match_score = self.target_person.get('match_score', 0.0)

                        # Check if re-acquired target has sufficient color match
                        if self.tracking_state == TrackingState.REACQUIRING:
                            if color_sim >= self.reacquire_threshold:
                                self.tracking_state = TrackingState.TRACKING
                                self.get_logger().info(f"Target re-acquired! (color: {color_sim:.2f})")
                            elif self.debug_logging:
                                self.get_logger().info(
                                    f"REACQUIRING: Color too low ({color_sim:.2f} < {self.reacquire_threshold})",
                                    throttle_duration_sec=1.0
                                )
                        else:
                            # Normal tracking - adaptively update histogram to handle pose changes
                            if self.adaptive_alpha > 0 and 'histogram' in self.target_person:
                                current_hist = self.target_person['histogram']
                                if current_hist is not None and self.target_color_histogram is not None:
                                    # Exponential moving average: blend current observation with stored histogram
                                    # This allows the system to adapt to pose changes (sitting→standing)
                                    # while remaining stable (not overreacting to noise)
                                    self.target_color_histogram = (
                                        self.adaptive_alpha * current_hist +
                                        (1.0 - self.adaptive_alpha) * self.target_color_histogram
                                    )

                            if self.debug_logging:
                                self.get_logger().info(
                                    f"TRACKING: Color={color_sim:.2f} Score={match_score:.2f} "
                                    f"Conf={self.target_person['confidence']:.2f}",
                                    throttle_duration_sec=2.0
                                )
                else:
                    # Fallback: no color tracking
                    self.target_person = max(persons, key=lambda p: p['confidence'])
                    self.tracking_state = TrackingState.TRACKING

            self.latest_detection = self.target_person

            # Publish target detection separately
            if self.target_person is not None:
                self.publish_target_detection(msg.detections, self.target_person)

            # Publish annotated image with target highlighted
            if self.current_image is not None:
                self.publish_tracking_image(msg.detections, persons, self.target_person)

            # Publish structured tracking data (for RQT dashboard)
            self.publish_tracking_data(persons, self.target_person)

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

        # Only follow when actively tracking (not when IDLE, LOST, or REACQUIRING)
        if self.tracking_state != TrackingState.TRACKING or self.target_person is None:
            # No active target, stop
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

    def publish_target_detection(self, all_detections, target_person):
        """Publish the target person's detection separately"""
        if target_person is None:
            return

        # Find the matching detection in the original message
        target_bbox = target_person['bbox']
        target_msg = Detection2DArray()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'camera_link'

        # Find the detection that matches our target
        for det in all_detections:
            det_x1 = det.bbox.center.x - det.bbox.size_x / 2.0
            det_y1 = det.bbox.center.y - det.bbox.size_y / 2.0
            det_x2 = det.bbox.center.x + det.bbox.size_x / 2.0
            det_y2 = det.bbox.center.y + det.bbox.size_y / 2.0

            # Check if this detection matches our target bbox
            if (abs(det_x1 - target_bbox[0]) < 1.0 and
                abs(det_y1 - target_bbox[1]) < 1.0 and
                abs(det_x2 - target_bbox[2]) < 1.0 and
                abs(det_y2 - target_bbox[3]) < 1.0):
                target_msg.detections.append(det)
                break

        if target_msg.detections:
            self.target_detection_pub.publish(target_msg)

    def draw_score_bar(self, image, bbox, match_score):
        """
        Draw dynamic score bar inside the top of the bounding box for target person
        Bar length and color change based on match score

        Args:
            image: Image to draw on (will be modified in place)
            bbox: Bounding box coordinates [x1, y1, x2, y2]
            match_score: Match score (0.0-1.0)
        """
        if match_score < 0:
            return  # Don't draw if no score available

        x1, y1, x2, y2 = [int(coord) for coord in bbox]
        box_width = x2 - x1

        # Bar dimensions
        bar_height = 6
        bar_y_offset = 5  # Pixels from top of bbox
        score_bar_y = y1 + bar_y_offset

        # Dark background
        cv2.rectangle(image, (x1 + 2, score_bar_y), (x2 - 2, score_bar_y + bar_height),
                     (0, 0, 0), -1)

        # Filled portion based on score (dynamic length)
        score_fill_width = int((box_width - 4) * match_score)

        if score_fill_width > 0:
            # Dynamic color based on score value
            if match_score >= 0.7:
                # High score - Green
                bar_color = (0, 255, 0)
            elif match_score >= 0.5:
                # Medium score - Yellow
                bar_color = (0, 255, 255)
            else:
                # Low score - Red
                bar_color = (0, 0, 255)

            cv2.rectangle(image, (x1 + 2, score_bar_y),
                         (x1 + 2 + score_fill_width, score_bar_y + bar_height),
                         bar_color, -1)

    def publish_tracking_image(self, all_detections, persons, target_person):
        """Publish annotated image with target highlighted in different color"""
        if self.current_image is None:
            return

        # Create annotated image
        annotated = self.current_image.copy()

        # Draw all persons (non-target) in green
        for person in persons:
            if person == target_person:
                continue  # Skip target, draw it last in different color

            bbox = person['bbox']
            x1, y1, x2, y2 = [int(coord) for coord in bbox]

            # Draw bounding box in GREEN
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label with ID
            person_id = person.get('id', '?')
            label = f"ID#{person_id}: {person['confidence']:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw target person in RED with thicker box and score bar
        if target_person is not None:
            bbox = target_person['bbox']
            x1, y1, x2, y2 = [int(coord) for coord in bbox]

            # Draw bounding box in RED with thicker border
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 4)  # RED, thicker

            # Draw dynamic score bar (color and length change based on match_score)
            match_score = target_person.get('match_score', -1.0)
            self.draw_score_bar(annotated, bbox, match_score)

            # Label with tracking info including ID
            person_id = target_person.get('id', '?')
            label = f"ID#{person_id} {target_person['confidence']:.2f}"
            if 'match_score' in target_person and target_person['match_score'] >= 0:
                label += f" | Score: {target_person['match_score']:.2f}"

            # Background for text
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x1, y1 - text_h - 10), (x1 + text_w, y1), (0, 0, 255), -1)
            cv2.putText(annotated, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = 'camera_link'
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish tracking image: {e}')

    def publish_tracking_data(self, persons, target_person):
        """
        Publish structured person tracking data for RQT dashboard
        """
        msg = PersonTrackingArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.tracking_state = self.tracking_state.name
        msg.target_id = self.target_person_id if self.target_person_id is not None else 0

        # Populate person tracking data
        for person in persons:
            person_msg = PersonTracking()
            person_msg.id = person.get('id', 0)
            person_msg.confidence = person.get('confidence', 0.0)

            # Bounding box
            bbox = person.get('bbox', [0, 0, 0, 0])
            person_msg.x1 = float(bbox[0])
            person_msg.y1 = float(bbox[1])
            person_msg.x2 = float(bbox[2])
            person_msg.y2 = float(bbox[3])

            # Color and match scores
            person_msg.color_similarity = person.get('color_similarity', -1.0)
            person_msg.match_score = person.get('match_score', -1.0)

            # Is target?
            person_msg.is_target = (person == target_person)

            # Distance and angle (only for target)
            if person == target_person:
                distance, angle = self.estimate_person_pose(person)
                person_msg.distance = distance if distance is not None else -1.0
                person_msg.angle = angle if angle is not None else 0.0
            else:
                person_msg.distance = -1.0
                person_msg.angle = 0.0

            msg.persons.append(person_msg)

        # Publish
        try:
            self.tracking_data_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish tracking data: {e}')


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
