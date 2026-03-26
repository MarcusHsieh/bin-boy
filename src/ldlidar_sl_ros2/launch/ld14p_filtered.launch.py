#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('ldlidar_sl_ros2').find('ldlidar_sl_ros2')

    # Path to laser filter config
    laser_filter_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'laser_filter_pillars.yaml'
    ])

    # Declare the launch argument for the serial port
    serial_port_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Serial port for the LDLIDAR'
    )

    # Use the launch configuration in the node
    port_name_config = LaunchConfiguration('port_name')

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14p',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14P'},
            {'laser_scan_topic_name': 'scan_raw'},  # Publish raw scan
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'base_laser'},
            {'port_name': port_name_config},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},  # Disabled - using laser_filters instead
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # Laser scan filter node to remove support pillar reflections
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_pillars',
        output='screen',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', 'scan_raw'),           # Input: raw scan from lidar
            ('scan_filtered', 'scan')       # Output: filtered scan for SLAM/Nav
        ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld14p',
        arguments=['0.0', '0', '0.2', '0', '0', '0', 'base_link', 'base_laser']
    )

    ld = LaunchDescription([
        serial_port_arg,
        ldlidar_node,
        laser_filter_node,
        base_link_to_laser_tf_node
    ])

    return ld
