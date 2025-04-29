import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

LIVOX_LOG_FILE = "/home/as/Desktop/ros_bags/livox.log"

def generate_launch_description():
    # Get the path to the Livox ROS Driver 2 launch file
    livox_launch_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2',
        'msg_HAP_launch.py'
    )

    # Include the Livox launch file
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path)
    )

    return LaunchDescription([
        livox_launch
    ])
