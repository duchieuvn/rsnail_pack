import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # sensors
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'sensors_launch.py'
            )
        )
    )

    # ros_bags
    ros_bags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'ros_bags_launch.py'
            )
        )
    )

    return LaunchDescription([
        sensors_launch,
        ros_bags_launch
    ])