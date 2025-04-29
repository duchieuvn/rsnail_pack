import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

CAMERA_LOG_FILE = "/home/as/Desktop/ros_bags/camera.log"

def generate_launch_description():
    # Get the path to the ZED wrapper launch file
    zed_launch_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    )

    # Declare the camera model argument
    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zed2i', description='ZED camera model'
    )

    # Include the ZED wrapper launch file with the camera_model argument
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_path),
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items()
    )

    return LaunchDescription([
        camera_model_arg,
        zed_launch
    ])

