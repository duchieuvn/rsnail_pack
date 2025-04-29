import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # can_driver
    can_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'can_driver_launch.py'
            )
        )
    )

    # can_decode
    can_decode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'can_decode_launch.py'
            )
        )
    )

    # can_controller
    can_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'can_controller_launch.py'
            )
        )
    )

    return LaunchDescription([
        can_driver_launch,
        can_decode_launch,
        can_controller_launch
    ])