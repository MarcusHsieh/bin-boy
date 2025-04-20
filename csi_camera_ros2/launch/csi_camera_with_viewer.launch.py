import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch the camera node and viewer node in a container for intra-process communication."""

    # Container to hold the nodes
    container = ComposableNodeContainer(
            name='csi_camera_container',
            namespace='',
            package='rclcpp_components', # Standard package for the container node
            executable='component_container',
            composable_node_descriptions=[
                # Camera Publisher Node
                ComposableNode(
                    package='csi_camera_ros2',
                    plugin='csi_camera_ros2.csi_camera_node:CSICameraNode', # Use colon separator
                    name='csi_camera_node',
                    parameters=[ # Pass default parameters explicitly if needed, or rely on node defaults
                        # Example: {'publish_raw': True, 'publish_compressed': False}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}] # Enable IPC
                ),
                # Image Viewer Node
                ComposableNode(
                    package='csi_camera_ros2',
                    plugin='csi_camera_ros2.image_viewer_node:ImageViewerNode', # Use colon separator
                    name='image_viewer_node',
                    # Remap if the viewer expects a different topic name (ours matches default)
                    # remappings=[('/image', '/csi_camera_0/image_raw')]
                    extra_arguments=[{'use_intra_process_comms': True}] # Enable IPC
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
