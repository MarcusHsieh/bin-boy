import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch C++ camera node and viewer node in composable container"""

    container = ComposableNodeContainer(
            name='csi_camera_container_cpp',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='csi_camera_cpp',
                    plugin='csi_camera_cpp::CSICameraNode',
                    name='csi_camera_node',
                    parameters=[
                         {'capture_width': 1920},
                         {'capture_height': 1080},
                         {'display_width': 960},
                         {'display_height': 540},
                         {'framerate': 30},
                         {'publish_rate': 30.0}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='csi_camera_cpp',
                    plugin='csi_camera_cpp::ImageViewerNode',
                    name='image_viewer_node',
                    remappings=[('image_raw', '/image_raw')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
