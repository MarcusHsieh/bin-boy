#!/usr/bin/env python3
"""
Waypoint Sender Node

Sends a sequence of waypoints to Nav2's FollowWaypoints action server.
Waypoints are loaded from a YAML file and sent as a PoseStamped array.

Usage:
    ros2 run bin_boy_navigation waypoint_sender <waypoint_file.yaml>

Example waypoint file format (patrol_square.yaml):
    waypoints:
      - {x: 1.0, y: 0.0, theta: 0.0}
      - {x: 1.0, y: 1.0, theta: 1.57}
      - {x: 0.0, y: 1.0, theta: 3.14}
      - {x: 0.0, y: 0.0, theta: -1.57}
"""

import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
import os


class WaypointSender(Node):
    """
    ROS2 node that sends waypoint sequences to Nav2's FollowWaypoints action server.
    """

    def __init__(self):
        super().__init__('waypoint_sender')

        # Create action client for FollowWaypoints
        self._action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')

        self.get_logger().info('Waypoint Sender node initialized')
        self.get_logger().info('Waiting for FollowWaypoints action server...')

    def load_waypoints_from_yaml(self, yaml_path):
        """
        Load waypoints from a YAML file.

        Args:
            yaml_path: Path to YAML file containing waypoint definitions

        Returns:
            List of waypoint dictionaries with x, y, theta keys
        """
        # Resolve package-relative paths (e.g., "config/waypoints/patrol.yaml")
        if not os.path.isabs(yaml_path):
            # Try to resolve as package-relative path
            try:
                pkg_share = get_package_share_directory('bin_boy_navigation')
                yaml_path = os.path.join(pkg_share, yaml_path)
                self.get_logger().info(f'Resolved relative path to: {yaml_path}')
            except Exception as e:
                self.get_logger().error(f'Could not resolve package path: {e}')
                return None

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'waypoints' not in data:
                self.get_logger().error('YAML file must contain "waypoints" key')
                return None

            waypoints = data['waypoints']
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {yaml_path}')
            return waypoints

        except FileNotFoundError:
            self.get_logger().error(f'Waypoint file not found: {yaml_path}')
            return None
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return None

    def create_pose_stamped(self, x, y, theta):
        """
        Create a PoseStamped message from x, y, theta coordinates.

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            theta: Orientation in radians

        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Convert theta to quaternion (rotation around Z axis)
        import math
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0

        return pose

    def send_waypoints(self, waypoint_file):
        """
        Send waypoints from file to the FollowWaypoints action server.

        Args:
            waypoint_file: Path to YAML file containing waypoints
        """
        # Load waypoints from YAML
        waypoints_data = self.load_waypoints_from_yaml(waypoint_file)
        if waypoints_data is None:
            return False

        # Wait for action server to be available
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowWaypoints action server not available after waiting 10 seconds')
            return False

        self.get_logger().info('FollowWaypoints action server is ready')

        # Convert waypoint data to PoseStamped messages
        poses = []
        for i, wp in enumerate(waypoints_data):
            if 'x' not in wp or 'y' not in wp or 'theta' not in wp:
                self.get_logger().error(f'Waypoint {i} missing required fields (x, y, theta)')
                return False

            pose = self.create_pose_stamped(wp['x'], wp['y'], wp['theta'])
            poses.append(pose)
            self.get_logger().info(f'Waypoint {i}: ({wp["x"]:.2f}, {wp["y"]:.2f}, {wp["theta"]:.2f} rad)')

        # Create goal message
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f'Sending {len(poses)} waypoints to FollowWaypoints action server...')

        # Send goal and set up callbacks
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        """
        Callback when goal is accepted/rejected by action server.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was REJECTED by action server')
            return

        self.get_logger().info('Waypoint goal ACCEPTED by action server')

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback for action feedback (progress updates).
        """
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'Progress: Navigating to waypoint {current_waypoint}')

    def result_callback(self, future):
        """
        Callback when action completes (success or failure).
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            missed = len(result.missed_waypoints)
            if missed == 0:
                self.get_logger().info('SUCCESS: All waypoints reached!')
            else:
                self.get_logger().warn(f'PARTIAL SUCCESS: {missed} waypoints were missed')
                for missed_wp in result.missed_waypoints:
                    self.get_logger().warn(f'  Missed waypoint at index {missed_wp}')
        else:
            self.get_logger().error(f'FAILED: Waypoint navigation failed with status {status}')

        # Shutdown node after completion
        self.get_logger().info('Waypoint sender shutting down')

        # Properly destroy action client before shutting down
        try:
            self._action_client.destroy()
        except Exception:
            pass  # Ignore if already destroyed

        rclpy.shutdown()


def main(args=None):
    """
    Main entry point for waypoint sender node.
    """
    rclpy.init(args=args)

    # Check for waypoint file argument
    if len(sys.argv) < 2:
        print('Usage: ros2 run bin_boy_navigation waypoint_sender <waypoint_file.yaml>')
        print('Example: ros2 run bin_boy_navigation waypoint_sender config/waypoints/patrol_square.yaml')
        sys.exit(1)

    waypoint_file = sys.argv[1]

    # Create node
    node = WaypointSender()

    # Send waypoints
    if not node.send_waypoints(waypoint_file):
        node.get_logger().error('Failed to send waypoints')
        sys.exit(1)

    # Spin to process callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Waypoint sender interrupted by user')

    node.destroy_node()


if __name__ == '__main__':
    main()
