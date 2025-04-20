import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch the C++ camera node and viewer node in a container for intra-process communication."""

    # Container to hold the nodes
    container = ComposableNodeContainer(
            name='csi_camera_container_cpp',
            namespace='',
            package='rclcpp_components', # Use the standard C++ container
            executable='component_container',
            composable_node_descriptions=[
                # Camera Publisher Node (C++)
                ComposableNode(
                    package='csi_camera_cpp',
                    plugin='csi_camera_cpp::CSICameraNode', # C++ plugin format namespace::ClassName
                    name='csi_camera_node', # Node name
                    parameters=[ # Set desired parameters here
                         {'capture_width': 1920},
                         {'capture_height': 1080},
                         {'display_width': 960},
                         {'display_height': 540},
                         {'framerate': 30},
                         {'publish_rate': 30.0}
                         # Add other parameters if needed, otherwise defaults are used
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}] # Enable IPC
                ),
                # Image Viewer Node (C++)
                ComposableNode(
                    package='csi_camera_cpp',
                    plugin='csi_camera_cpp::ImageViewerNode', # C++ plugin format namespace::ClassName
                    name='image_viewer_node', # Node name
                    # Remapping needed because viewer subscribes to relative 'image_raw'
                    # The topic will be /csi_camera_node/image_raw if publisher name is csi_camera_node
                    remappings=[('image_raw', '/csi_camera_node/image_raw')],
                    extra_arguments=[{'use_intra_process_comms': True}] # Enable IPC
                ),
            ],
            output='screen',
            # Add this if you need to see DEBUG logs from nodes
            # arguments=['--ros-args', '--log-level', 'debug'],
    )

    return launch.LaunchDescription([container])
