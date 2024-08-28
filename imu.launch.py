import os
import ament_index_python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 
                                       'launch', 'microstrain_launch.py')
IMU_CONFIG_FILE = os.path.join('/home/orl/Documents/camera_calibration', 'imu_cv7.yml')

def generate_launch_description():
    return LaunchDescription([
        # Microstrain IMU Driver Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(MICROSTRAIN_LAUNCH_FILE),
            launch_arguments={
                'configure': 'true',
                'activate': 'true',
                'params_file': IMU_CONFIG_FILE,
                'namespace': '/',
            }.items()
        ),
        # IMU to Robot Base Transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link"
                ]
        ),
    ])
