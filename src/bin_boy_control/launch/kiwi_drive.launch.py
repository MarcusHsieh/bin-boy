#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for motor driver board'
        ),

        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Serial baudrate'
        ),

        Node(
            package='bin_boy_control',
            executable='kiwi_drive_node',
            name='kiwi_drive_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'wheel_ids': [1, 2, 3],
                'wheel_radius': 0.113,  # Calibrated: was 0.07, adjusted by factor 1.6155 (1m actual / 0.619m odom)
                'robot_radius': 0.15,
                'publish_rate': 50.0,
                'publish_tf': True,
                'encoder_resolution': 4096
            }]
        )
    ])
