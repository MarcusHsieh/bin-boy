#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('bin_boy_control')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'full_control.rviz')

    return LaunchDescription([
        # Launch arguments for kiwi drive
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for motor driver board'
        ),

        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Serial baudrate'
        ),

        # Launch arguments for IMU
        DeclareLaunchArgument(
            'imu_i2c_bus',
            default_value='1',
            description='I2C bus number for MPU6050'
        ),

        DeclareLaunchArgument(
            'imu_i2c_address',
            default_value='0x68',
            description='I2C address for MPU6050 (0x68 or 0x69)'
        ),

        DeclareLaunchArgument(
            'imu_publish_rate',
            default_value='100.0',
            description='IMU publish rate in Hz'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),

        # Kiwi Drive Node
        Node(
            package='bin_boy_control',
            executable='kiwi_drive_node',
            name='kiwi_drive_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'wheel_ids': [1, 2, 3],
                'wheel_radius': 0.05,
                'robot_radius': 0.15,
                'publish_rate': 50.0,
                'publish_tf': True,
                'encoder_resolution': 4096
            }]
        ),

        # MPU6050 IMU Node
        Node(
            package='bin_boy_control',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            parameters=[{
                'i2c_bus': LaunchConfiguration('imu_i2c_bus'),
                'i2c_address': 0x68,  # Using 0x68 (default address, AD0 low/floating)
                'publish_rate': LaunchConfiguration('imu_publish_rate'),
                'frame_id': 'imu_link'
            }]
        ),

        # Robot Localization EKF Node (fuses odometry + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', 'odom_filtered')
            ]
        ),

        # RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
