from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Left camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='left_camera',
            namespace='stereo/left',
            parameters=[{
                'video_device': '/dev/video4',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/left',
                'camera_info_url': 'file:///home/orl/Documents/camera_calibration/calibration_left.yaml',
                'frame_id': 'camera_left'
            }]
        ),
        # Right camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='right_camera',
            namespace='stereo/right',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'narrow_stereo/right',
                'camera_info_url': 'file:///home/orl/Documents/camera_calibration/calibration_right.yaml',
                'frame_id': 'camera_right'
            }]
        ),
        #   Image rectification container
        ComposableNodeContainer(
            name='image_proc_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='stereo',
            composable_node_descriptions= [
                # Separate mono and color for the left camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name='debayer',
                    namespace='stereo/left'
                ),
                # Separate mono and color for the right camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name='debayer',
                    namespace='stereo/right'
                ),
                # Mono image rectification for the left camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_mono',
                    namespace='stereo/left',
                    remappings=[
                        ('image', 'image_mono'),
                        ('image_rect', 'image_rect')
                    ],
                ),
                # Mono image rectification for the right camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_mono',
                    namespace='stereo/right',
                    remappings=[
                        ('image', 'image_mono'),
                        ('image_rect', 'image_rect')
                    ],
                ),
                # Color image rectification for the left camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_color',
                    namespace='stereo/left',
                    remappings=[
                        ('image', 'image_color'),
                        ('image_rect', 'image_rect_color')
                    ],
                ),
                # Color image rectification for the right camera
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_color',
                    namespace='stereo/right',
                    remappings=[
                        ('image', 'image_color'),
                        ('image_rect', 'image_rect_color')
                    ],
                ),
                # Disparity Map
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    namespace='stereo',
                    parameters=[{
                        'approximate_sync': True
                    }]
                ),
                # Point cloud 
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    namespace='stereo',
                    parameters=[{
                        'approximate_sync': True,
                        'use_color': True
                    }]
                ),
            ]
        ),
        # Left camera static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_left_camera_to_camera',
            arguments=['0', '0.04', '0.029', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_left']
        ),
        # Right camera static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_right_camera_to_camera',
            arguments=['0', '-0.04', '0.029', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_right']
        ),
        # Camera link static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_right_camera_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])
