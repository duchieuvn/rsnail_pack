from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # QT CAN Bus Sender
    # Replace this with the appropriate launch file if available
    qtcanbus_sender_sensor = Node(
        package="ros2_qtcanbus",
        executable="qtcanbus_sender",
        name="qtcanbus_sender",
        parameters=[
            {"canbus_plugin": "peakcan", "canbus_interface": "usb0", "can_type": "sensor"}
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )

    qtcanbus_sender_motor = Node(
        package="ros2_qtcanbus",
        executable="qtcanbus_sender",
        name="qtcanbus_sender",
        parameters=[
            {"canbus_plugin": "peakcan", "canbus_interface": "usb1", "can_type": "motor"}                  
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )

    return LaunchDescription(
        [
            qtcanbus_sender_sensor,
            qtcanbus_sender_motor,
        ]
    )



