from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="delaunay_pipeline",
                executable="delauney_pipeline_node",
                name="delauney_pipeline_node"
            )
        ]
    )
