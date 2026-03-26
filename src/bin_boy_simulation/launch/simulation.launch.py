#!/usr/bin/env python3
"""
Launch Gazebo simulation with bin_boy robot

Mapping options:
  slam:=true - Dynamic SLAM mapping (best for new/changing environments)
  localization:=true - Static map localization with AMCL (best for known environments)
  (default: neither - basic odometry only)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
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

    # Try to get navigation package (may not be available)
    try:
        pkg_navigation = get_package_share_directory('bin_boy_navigation')
        nav_available = True
    except:
        nav_available = False

    # Paths
    world_file = PathJoinSubstitution([pkg_simulation, 'worlds', 'test_indoor.world'])
    urdf_file = PathJoinSubstitution([pkg_simulation, 'urdf', 'bin_boy_gazebo.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_simulation, 'rviz', 'simulation.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    world = LaunchConfiguration('world', default=world_file)
    enable_localization = LaunchConfiguration('localization', default='false')
    enable_slam = LaunchConfiguration('slam', default='false')

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

    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Enable AMCL localization with map server (requires bin_boy_navigation package)'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Enable SLAM Toolbox for dynamic mapping (best for person following)'
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

    # Localization/SLAM nodes (optional - only if bin_boy_navigation is available)
    mapping_nodes = []
    if nav_available:
        # Config files
        nav2_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
        amcl_params = os.path.join(pkg_navigation, 'config', 'amcl_params.yaml')
        slam_config = os.path.join(pkg_simulation, 'config', 'slam_toolbox.yaml')

        # AMCL Localization (static map)
        # Map Server
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(enable_localization)
        )

        # AMCL Localization
        amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params],
            condition=IfCondition(enable_localization)
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
            ],
            condition=IfCondition(enable_localization)
        )

        # Delay lifecycle_manager to ensure map_server and AMCL are ready
        delayed_localization = TimerAction(
            period=3.0,
            actions=[lifecycle_manager_localization]
        )

        # SLAM Toolbox (dynamic mapping)
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

        mapping_nodes = [map_server, amcl, delayed_localization, slam_node]

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        declare_world,
        declare_localization,
        declare_slam,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_robot,
        rviz_node
    ] + mapping_nodes)
