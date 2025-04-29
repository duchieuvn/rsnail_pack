import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = "odom_preprocessing"
  config_dir = get_package_share_directory(package_name)

  # Set the path to different files and folders.
  robot_localization_file_path = os.path.join(config_dir, 'ekf.yaml')
  pulish_acceleration=True
   # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path])
 
  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(start_robot_localization_cmd)

  return ld
