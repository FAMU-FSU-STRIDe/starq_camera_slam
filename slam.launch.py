from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        #  SLAM container
        ComposableNodeContainer(
            name='slam_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='slam',
            composable_node_descriptions= [
                ComposableNode(
                    name='visual_slam_node',
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    parameters=[{
                        'rectified_images': True,
                        'enable_imu_fusion': False,
                        'debug_imu_mode': False,
                        'enable_debug_mode': False,
                        'debug_dump_path': 'debug',
                        'enable_image_denoising': True,
                        'enable_slam_visualization': True, # visualization may affects the performance
                        'enable_landmarks_view': True, # visualization may affects the performance
                        'enable_observations_view': False, # visualization may affects the performance
                        'enable_localization_n_mapping': True,
                        'image_jitter_threshold_ms': 35.00, # for 30 FPS
                        'sync_matching_threshold_ms': 20.0, # desync in ms between different cams
                        'num_cameras': 2, # 2 cams within one stereo camera
                        'base_frame': 'camera_link',
                        'verbocity': 3,
                        'image_buffer_size': 200,
                    }],
                    remappings=[('visual_slam/image_0', '/stereo/left/image_rect'),
                                ('visual_slam/camera_info_0', '/stereo/left/camera_info'),
                                ('visual_slam/image_1', '/stereo/right/image_rect'),
                                ('visual_slam/camera_info_1', '/stereo/right/camera_info')
                    ]
                )
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/orl/Documents/camera_calibration/slam_rviz.rviz']  # Load config file
        ),
    ])
