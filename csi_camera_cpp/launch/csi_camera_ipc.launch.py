import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    """Called by OpaqueFunction to generate launch description based on arguments."""
    run_detector = LaunchConfiguration('run_detector').perform(context).lower() == 'true'
    detection_frame_skip = int(LaunchConfiguration('detection_frame_skip').perform(context))
    publish_annotated_image = LaunchConfiguration('publish_annotated_image').perform(context).lower() == 'true'

    # parameters
    camera_node_params = [
        {'capture_width': 1280},
        {'capture_height': 720},
        {'display_width': 640},
        {'display_height': 480},
        {'framerate': 15},
        {'publish_rate': 15.0}
    ]

    # nodes camera + viewer
    composable_nodes = [
        ComposableNode(
            package='csi_camera_cpp',
            plugin='csi_camera_cpp::CSICameraNode',
            name='csi_camera_node',
            parameters=camera_node_params,
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        ComposableNode(
            package='csi_camera_cpp',
            plugin='csi_camera_cpp::ImageViewerNode',
            name='image_viewer_node',
            remappings=[('image_raw', '/person_detections/image' if run_detector and publish_annotated_image else '/image_raw')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
    ]

    # node detector
    if run_detector:
        composable_nodes.append(
            ComposableNode(
                package='csi_camera_cpp',
                plugin='csi_camera_cpp::PersonDetectorNode',
                    name='person_detector_node',
                    remappings=[('image_raw', '/image_raw')],
                    parameters=[
                        {'detection_frame_skip': detection_frame_skip}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
            )
        )

    # container
    container = ComposableNodeContainer(
            name='csi_camera_container_cpp',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=composable_nodes,
            output='screen',
    )

    return [container]


def generate_launch_description():
    """Generates launch description with arguments for conditional detector loading."""
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'run_detector',
            default_value='true',
            description='Whether to run person detector node'
        ),
        DeclareLaunchArgument(
            'detection_frame_skip',
            default_value='1',
            description='Number of frames to skip between detections (0=detect every frame)'
        ),
         DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='false',
            description='Whether detector node should publish annotated image'
        ),
        OpaqueFunction(function=launch_setup)
    ])
