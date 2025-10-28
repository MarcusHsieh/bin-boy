#!/usr/bin/env python3
"""
SLAM Simulation Launch
Launches complete system with SLAM Toolbox for 2D mapping

This launch file brings together:
- Gazebo simulation with robot, sensors, and environment
- EKF sensor fusion (odometry + IMU)
- SLAM Toolbox for 2D mapping from lidar data
- Optional person tracking and following
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('bin_boy_simulation')

    # Paths
    slam_config = PathJoinSubstitution([pkg_simulation, 'config', 'slam_toolbox.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_slam = LaunchConfiguration('enable_slam', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM Toolbox mapping'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # Include full system simulation (Gazebo + Robot + Sensors + EKF)
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'full_system_sim.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': enable_rviz
        }.items()
    )

    # SLAM Toolbox - Synchronous SLAM for mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_slam)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_slam,
        declare_enable_rviz,
        full_system_launch,
        slam_node
    ])
