import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_launch_path = os.path.join(
        get_package_share_directory('landmark_slam_2d'),
        'launch',
        'slam.launch.py'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path)
    )

    return LaunchDescription([
        slam_launch
    ])
