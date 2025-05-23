from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # QT CAN Bus Sender
    # Replace this with the appropriate launch file if available
    #qtcanbus_sender = Node(
    #    package="ros2_qtcanbus",
    #    executable="qtcanbus_sender",
    #    name="qtcanbus_sender",
    #    parameters=[
    #        {"canbus_plugin": "clx000can", "canbus_interface": "cu.usbmodem123456781"}
    #    ],
    #    arguments=["--ros-args", "--log-level", "debug"],
    #)

    # Sensor and motor can nodes
    ros2_can_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros2_candecode"),
                    "launch",
                    "can_nodes_launch.py",
                )
            ]
        )
    )


    # ROS2 CAN Decode
    # Adjust this path to the actual launch file location
    ros2_candecode_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros2_candecode"),
                    "launch",
                    "standalone_launch.py",
                )
            ]
        )
    )

    # Velodyne
    velodyne_launch_file = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           [
               os.path.join(
                   get_package_share_directory("velodyne"),
                   "launch",
                   "velodyne-all-nodes-VLP16-launch.py",
               )
           ]
       )
    )

    # ROS Bag Generation
    # Generates Ros Bags under the home directory
    ros_bag_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros2_candecode"),
                    "launch",
                    "ros_bag_launch.py",
                )
            ]
        )
    )


    return LaunchDescription(
        [
            #qtcanbus_sender,
            ros2_candecode_launch_file,
            ros2_can_interface,
            #velodyne_launch_file,
            ros_bag_launch_file,
        ]
    )
