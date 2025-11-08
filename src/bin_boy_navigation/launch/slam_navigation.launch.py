#!/usr/bin/env python3
"""
SLAM + Navigation Launch File
Launches complete system: SLAM Toolbox + Nav2 stack

This enables simultaneous mapping and navigation.
Use this when exploring new environments while autonomously navigating.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('bin_boy_simulation')
    pkg_navigation = get_package_share_directory('bin_boy_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # Include SLAM simulation (Gazebo + SLAM Toolbox)
    slam_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'slam_sim.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': enable_rviz
        }.items()
    )

    # Include Navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_navigation, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_rviz,
        slam_sim_launch,
        navigation_launch
    ])
