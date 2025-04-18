import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # can_pkg
    can_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'can_pkg_launch.py'
            )
        )
    )

    # camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'camera_launch.py'
            )
        )
    )

    # lidars
    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'lidars_launch.py'
            )
        )
    )

    # ekf
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'ekf_launch.py'
            )
        )
    )

    # perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'perception_launch.py'
            )
        )
    )

    # slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'slam_launch.py'
            )
        )
    )

    # slam
    path_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('launch_files'),
                'launch',
                'path_planning_launch.py'
            )
        )
    )

    return LaunchDescription([
        can_pkg_launch,
        camera_launch,
        lidars_launch,
        ekf_launch,
        perception_launch,
        slam_launch,
        path_planning_launch,
    ])
