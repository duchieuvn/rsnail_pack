import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

VELODYNE_LOG_FILE = "/home/as/Desktop/ros_bags/velodyne.log"

def generate_launch_description():
    # Get the path to the Velodyne launch file
    velodyne_launch_path = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP16-launch.py'
    )

    # Include the Velodyne launch file
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path)
    )

    return LaunchDescription([
        velodyne_launch
    ])
