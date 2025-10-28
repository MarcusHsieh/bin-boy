#!/usr/bin/env python3
"""
Simulation Person Tracking Launch File

Launches mock person detector + person tracker for testing in Gazebo simulation.
This replaces the real camera + YOLOv5 detector with a mock detector that uses
Gazebo ground truth.

Usage:
  ros2 launch bin_boy_perception sim_person_tracking.launch.py enable_following:=true
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for simulation person tracking
    """

    # Launch arguments
    enable_following_arg = DeclareLaunchArgument(
        'enable_following',
        default_value='false',
        description='Enable robot following behavior (publishes cmd_vel)'
    )

    enable_color_tracking_arg = DeclareLaunchArgument(
        'enable_color_tracking',
        default_value='false',
        description='Enable color-based person re-identification (disabled for sim)'
    )

    min_confidence_arg = DeclareLaunchArgument(
        'min_confidence',
        default_value='0.5',
        description='Minimum detection confidence threshold (0.0-1.0)'
    )

    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.5',
        description='Target following distance in meters'
    )

    debug_logging_arg = DeclareLaunchArgument(
        'debug_logging',
        default_value='false',
        description='Enable verbose debug logging'
    )

    # Mock person detector node
    mock_detector_node = Node(
        package='bin_boy_perception',
        executable='mock_person_detector',
        name='mock_person_detector',
        output='screen',
        parameters=[{
            'person_model_prefix': 'person',  # Look for models named "person1", "person2", etc.
            'person_height': 1.7,
            'person_radius': 0.3,
            'detection_confidence': 0.95,
            'robot_name': 'bin_boy',
            'camera_frame': 'camera_optical_frame',
            'debug_logging': LaunchConfiguration('debug_logging'),
        }]
    )

    # Person tracker node
    person_tracker_node = Node(
        package='bin_boy_perception',
        executable='person_tracker',
        name='person_tracker',
        output='screen',
        parameters=[{
            'detection_topic': '/person_detections',
            'image_topic': '/camera/image_raw',  # From Gazebo camera
            'camera_info_topic': '/camera/camera_info',  # From Gazebo camera
            'min_confidence': LaunchConfiguration('min_confidence'),
            'target_distance': LaunchConfiguration('target_distance'),
            'max_tracking_distance': 5.0,
            'enable_following': LaunchConfiguration('enable_following'),
            'enable_color_tracking': LaunchConfiguration('enable_color_tracking'),
            'lost_timeout': 3.0,
            'reacquire_color_threshold': 0.6,
            'adaptive_histogram_alpha': 0.4,
            'debug_logging': LaunchConfiguration('debug_logging'),
        }],
        remappings=[
            ('/person_detections', '/person_detections'),
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        # Launch arguments
        enable_following_arg,
        enable_color_tracking_arg,
        min_confidence_arg,
        target_distance_arg,
        debug_logging_arg,

        # Nodes
        mock_detector_node,
        person_tracker_node,
    ])
