#!/usr/bin/env python3
"""
Full System Simulation Launch
Launches complete system with all sensors, localization, and visualization
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
    pkg_control = get_package_share_directory('bin_boy_control')

    # Paths
    ekf_config = PathJoinSubstitution([pkg_control, 'config', 'ekf.yaml'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_localization = LaunchConfiguration('enable_localization', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Enable robot_localization EKF'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )

    # Include base simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'true',
            'rviz': enable_rviz
        }.items()
    )

    # Robot localization (EKF) - fuses odometry + IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_localization)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_enable_localization,
        declare_enable_rviz,
        simulation_launch,
        ekf_node
    ])
