#!/usr/bin/env python3
"""
Navigation Launch File
Launches Nav2 stack for autonomous navigation

This launch file starts all Nav2 nodes:
- controller_server (DWB holonomic controller)
- planner_server (NavFn planner)
- behavior_server (recovery behaviors)
- bt_navigator (behavior tree executor)
- waypoint_follower (multi-waypoint navigation)
- lifecycle_manager (manages node lifecycle)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_nav = get_package_share_directory('bin_boy_navigation')

    # Paths to config files
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Full path to nav2_params.yaml file'
    )

    # Set environment variables
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Recoveries Server (Recovery Behaviors)
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Lifecycle Manager (delayed start to let other nodes initialize)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                        'waypoint_follower'
                    ]}]
    )

    # Delay lifecycle_manager to ensure other nodes are ready
    delayed_lifecycle_manager = TimerAction(
        period=5.0,
        actions=[lifecycle_manager]
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        waypoint_follower,
        delayed_lifecycle_manager
    ])
