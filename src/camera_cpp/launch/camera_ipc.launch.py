import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    """Called by OpaqueFunction to generate launch description based on arguments."""
    camera_type = LaunchConfiguration('camera_type').perform(context)
    run_detector = LaunchConfiguration('run_detector').perform(context).lower() == 'true'
    detection_frame_skip = int(LaunchConfiguration('detection_frame_skip').perform(context))
    publish_annotated_image = LaunchConfiguration('publish_annotated_image').perform(context).lower() == 'true'
    confidence_threshold = float(LaunchConfiguration('confidence_threshold').perform(context))

    # Common camera parameters
    camera_node_params = [
        {'camera_type': camera_type},
        {'capture_width': 1280},
        {'capture_height': 720},
        {'display_width': 640},
        {'display_height': 480},
        {'framerate': 15},
        {'publish_rate': 15.0}
    ]

    # Add CSI-specific parameters if using CSI camera
    if camera_type == 'csi':
        camera_node_params.extend([
            {'sensor_id': int(LaunchConfiguration('sensor_id').perform(context))},
            {'flip_method': int(LaunchConfiguration('flip_method').perform(context))},
            {'awb_mode': int(LaunchConfiguration('awb_mode').perform(context))},
            {'crop_left': float(LaunchConfiguration('crop_left').perform(context))},
            {'crop_right': float(LaunchConfiguration('crop_right').perform(context))},
            {'crop_top': float(LaunchConfiguration('crop_top').perform(context))},
            {'crop_bottom': float(LaunchConfiguration('crop_bottom').perform(context))},
            {'enable_distortion_correction': LaunchConfiguration('enable_distortion_correction').perform(context).lower() == 'true'},
            {'use_center_crop_only': LaunchConfiguration('use_center_crop_only').perform(context).lower() == 'true'},
            {'center_crop_percentage': float(LaunchConfiguration('center_crop_percentage').perform(context))},
            {'barrel_distortion_k1': float(LaunchConfiguration('barrel_distortion_k1').perform(context))}
        ])
    # Add USB-specific parameters if using USB camera
    elif camera_type == 'usb':
        camera_node_params.append(
            {'device_id': int(LaunchConfiguration('device_id').perform(context))}
        )

    # Unified camera node (supports both CSI and USB)
    composable_nodes = [
        ComposableNode(
            package='camera_cpp',
            plugin='camera_cpp::CameraNode',
            name='camera_node',
            parameters=camera_node_params,
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        # ImageViewerNode (commented out by default, uncomment to enable live preview)
        # ComposableNode(
        #     package='camera_cpp',
        #     plugin='camera_cpp::ImageViewerNode',
        #     name='image_viewer_node',
        #     remappings=[('image_raw', '/person_detections/image' if run_detector and publish_annotated_image else '/image_raw')],
        #     extra_arguments=[{'use_intra_process_comms': True}]
        # ),
    ]

    # Add person detector node if enabled
    if run_detector:
        composable_nodes.append(
            ComposableNode(
                package='camera_cpp',
                plugin='camera_cpp::PersonDetectorNode',
                name='person_detector_node',
                remappings=[('image_raw', '/image_raw')],
                parameters=[
                    {'detection_frame_skip': detection_frame_skip},
                    {'confidence_threshold': confidence_threshold},
                    {'publish_annotated_image': publish_annotated_image}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        )

    # Composable node container with IPC enabled
    container = ComposableNodeContainer(
        name='camera_container_cpp',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return [container]


def generate_launch_description():
    """Generates launch description with arguments for camera type and optional detector."""
    return launch.LaunchDescription([
        # Camera type selection (REQUIRED)
        DeclareLaunchArgument(
            'camera_type',
            default_value='usb',
            description='Camera type: "csi" for CSI ribbon camera, "usb" for USB camera'
        ),

        # CSI camera parameters (only used if camera_type=csi)
        DeclareLaunchArgument(
            'sensor_id',
            default_value='0',
            description='CSI camera sensor ID (default: 0)'
        ),
        DeclareLaunchArgument(
            'flip_method',
            default_value='0',
            description='CSI camera flip method (0=none, 1=ccw90, 2=180, 3=cw90, 4=horizontal, 5=vertical)'
        ),
        DeclareLaunchArgument(
            'awb_mode',
            default_value='1',
            description='Auto white balance mode (0=off, 1=auto, 2=incandescent, 3=fluorescent, 5=daylight, 8=cloudy)'
        ),
        DeclareLaunchArgument(
            'crop_left',
            default_value='0.00',
            description='Crop percentage from left edge (0.0-1.0, removes vignetting) - Calibrated: 0.00'
        ),
        DeclareLaunchArgument(
            'crop_right',
            default_value='0.00',
            description='Crop percentage from right edge (0.0-1.0, removes vignetting) - Calibrated: 0.00'
        ),
        DeclareLaunchArgument(
            'crop_top',
            default_value='0.00',
            description='Crop percentage from top edge (0.0-1.0, removes vignetting) - Calibrated: 0.00'
        ),
        DeclareLaunchArgument(
            'crop_bottom',
            default_value='0.00',
            description='Crop percentage from bottom edge (0.0-1.0, removes vignetting) - Calibrated: 0.00'
        ),
        DeclareLaunchArgument(
            'enable_distortion_correction',
            default_value='true',
            description='Enable barrel distortion correction for ultra-wide lens (true/false) - Calibrated: true'
        ),
        DeclareLaunchArgument(
            'use_center_crop_only',
            default_value='false',
            description='Use only center portion instead of correction (true/false, faster but loses FOV) - Calibrated: false'
        ),
        DeclareLaunchArgument(
            'center_crop_percentage',
            default_value='0.70',
            description='If center crop only, percentage to keep (0.0-1.0, 0.70=center 70%)'
        ),
        DeclareLaunchArgument(
            'barrel_distortion_k1',
            default_value='-0.130',
            description='Radial distortion coefficient k1 (negative=barrel) - Calibrated for Waveshare IMX219-200'
        ),

        # USB camera parameters (only used if camera_type=usb)
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='USB camera device ID (0=/dev/video0, 1=/dev/video1, etc.)'
        ),

        # Person detector parameters
        DeclareLaunchArgument(
            'run_detector',
            default_value='true',
            description='Whether to run person detector node'
        ),
        DeclareLaunchArgument(
            'detection_frame_skip',
            default_value='1',
            description='Number of frames to skip between detections (0=detect every frame, 1=every 2nd frame)'
        ),
        DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='false',
            description='Whether detector node should publish annotated image with bounding boxes'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='Confidence threshold for detections (0.0-1.0, higher = fewer false positives)'
        ),

        OpaqueFunction(function=launch_setup)
    ])
