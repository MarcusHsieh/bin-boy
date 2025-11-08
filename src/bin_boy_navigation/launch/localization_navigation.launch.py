#!/usr/bin/env python3
"""
Localization + Navigation Launch File
Launches Nav2 stack with AMCL localization (uses pre-built map)

Use this mode when you have a saved map and want to navigate without SLAM.
For mapping, use slam_navigation.launch.py instead.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('bin_boy_simulation')
    pkg_navigation = get_package_share_directory('bin_boy_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_rviz = LaunchConfiguration('rviz', default='true')
    map_file = LaunchConfiguration('map', default='')

    # Config files
    nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    amcl_params = os.path.join(pkg_navigation, 'config', 'amcl_params.yaml')
    rviz_config = os.path.join(pkg_simulation, 'rviz', 'simulation.rviz')

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

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map YAML file (if empty, uses nav2_params.yaml default)'
    )

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Include full system simulation (Gazebo + Robot + Sensors + EKF, without SLAM)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_simulation, 'launch', 'full_system_sim.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': enable_rviz  # RViz will use Transient Local QoS (fixed in simulation.rviz)
        }.items()
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}  # Override if provided
        ]
    )

    # AMCL Localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params]
    )

    # Lifecycle Manager for map_server and AMCL
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
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

    # Delayed start for map_server and AMCL
    delayed_localization = TimerAction(
        period=3.0,
        actions=[lifecycle_manager_localization]
    )

    # Delayed start for navigation stack (wait for AMCL to initialize transform)
    delayed_navigation = TimerAction(
        period=8.0,
        actions=[navigation_launch]
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_use_sim_time,
        declare_enable_rviz,
        declare_map_file,
        simulation_launch,
        map_server,
        amcl,
        delayed_localization,
        delayed_navigation
    ])
