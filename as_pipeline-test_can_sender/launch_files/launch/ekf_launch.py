import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #Launch EKF state estimation
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('odom_preprocessing'),
                'state_estimation_launch.py'
            )
        )
    )

    return LaunchDescription([
        ekf_launch
    ])
