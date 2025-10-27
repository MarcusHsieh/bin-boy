#!/usr/bin/env python3
"""
Launch Gazebo simulation with bin_boy robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_simulation = get_package_share_directory('bin_boy_simulation')
    pkg_description = get_package_share_directory('bin_boy_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    world_file = PathJoinSubstitution([pkg_simulation, 'worlds', 'test_indoor.world'])
    urdf_file = PathJoinSubstitution([pkg_simulation, 'urdf', 'bin_boy_gazebo.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_description, 'rviz', 'display.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    world = LaunchConfiguration('world', default=world_file)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI if true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )

    # Process URDF
    robot_description_content = Command([
        'xacro ', urdf_file
    ])

    # Wrap in ParameterValue to avoid YAML parsing issues with XML content
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )

    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
        condition=IfCondition(gui)
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_bin_boy',
        output='screen',
        arguments=[
            '-entity', 'bin_boy',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',  # Spawn on ground, wheels will touch properly
            '-Y', '0.0'
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    # No additional controller needed!
    # The planar_move plugin in the URDF handles everything:
    # - Subscribes to /cmd_vel
    # - Publishes /odom
    # - Keeps robot stable (locks Z, roll, pitch)
    # - Allows planar motion (X, Y, yaw)

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        declare_world,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_robot,
        rviz_node
    ])
