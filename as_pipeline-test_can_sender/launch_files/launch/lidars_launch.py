import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'livox_launch.py'
            )
        )
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'velodyne_launch.py'
            )
        )
    )

    transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'transformer_launch.py'
            )
        )
    )

    return LaunchDescription([
        livox_launch,
        velodyne_launch,
        transform_launch
    ])