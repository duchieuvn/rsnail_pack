from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#logger = LaunchConfiguration("log-level")

config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_candecode"), "config", "candecode.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log-level",
                default_value=["debug"],
                description="Logging level",
            ),
            Node(
                package="ros2_candecode",
                executable="candecode",
                name="candecode_node",
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", "debug"]
            ),
        ]
    )
