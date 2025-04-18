from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lidar_pipeline",
                executable="lidar_pipeline_node",
                name="lidar_pipeline_node"
            ),
            Node(
                package="perception_filter",
                executable="perception_filter_node",
                name="perception_filter_node"
            )
        ]
    )
