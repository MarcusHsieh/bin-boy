#!/usr/bin/env python3
"""
Person Tracking Launch File

Launches the complete person tracking pipeline:
- Camera node (CSI/USB)
- Person detector (YOLOv5 TensorRT)
- Person tracker (color-based re-identification + following)

Example usage:
  # CSI camera with person tracking and following
  ros2 launch bin_boy_perception person_tracking.launch.py camera_type:=csi enable_following:=true

  # USB camera with tracking only (no movement)
  ros2 launch bin_boy_perception person_tracking.launch.py camera_type:=usb enable_following:=false
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate launch description for complete person tracking system
    """

    # Package directories
    camera_pkg = get_package_share_directory('camera_cpp')

    # Launch arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='usb',
        description='Camera type: csi or usb'
    )

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='USB camera device ID (only for USB cameras)'
    )

    enable_following_arg = DeclareLaunchArgument(
        'enable_following',
        default_value='false',
        description='Enable robot following behavior (publishes cmd_vel)'
    )

    enable_color_tracking_arg = DeclareLaunchArgument(
        'enable_color_tracking',
        default_value='false',
        description='Enable color-based person re-identification'
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

    # Include camera + detector launch file
    # Note: camera_cpp has nested launch/launch directory structure
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_pkg, 'launch', 'launch', 'camera_ipc.launch.py')
        ),
        launch_arguments={
            'camera_type': LaunchConfiguration('camera_type'),
            'device_id': LaunchConfiguration('device_id'),
            'enable_distortion_correction': 'false',  # Disabled for OV9281 (148° FOV, minimal distortion)
            'run_detector': 'true',
            'publish_annotated_image': 'true',  # Enable bounding box visualization
        }.items()
    )

    # Person tracker node
    person_tracker_node = Node(
        package='bin_boy_perception',
        executable='person_tracker',
        name='person_tracker',
        output='screen',
        parameters=[{
            'detection_topic': '/person_detections',
            'image_topic': '/image_raw',
            'camera_info_topic': '/camera_info',
            'min_confidence': LaunchConfiguration('min_confidence'),
            'target_distance': LaunchConfiguration('target_distance'),
            'max_tracking_distance': 5.0,
            'enable_following': LaunchConfiguration('enable_following'),
            'enable_color_tracking': LaunchConfiguration('enable_color_tracking'),
            'lost_timeout': 3.0,
            'reacquire_color_threshold': 0.6,
            'adaptive_histogram_alpha': 0.4,  # Faster adaptation to pose changes (sit/stand)
        }],
        remappings=[
            ('/person_detections', '/person_detections'),
            ('/image_raw', '/image_raw'),
            ('/camera_info', '/camera_info'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        # Launch arguments
        camera_type_arg,
        device_id_arg,
        enable_following_arg,
        enable_color_tracking_arg,
        min_confidence_arg,
        target_distance_arg,

        # Nodes
        camera_launch,
        person_tracker_node,
    ])
