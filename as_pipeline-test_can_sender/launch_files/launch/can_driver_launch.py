from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # QT CAN Bus Sender
    qtcanbus_sender_sensor = Node(
        package="ros2_qtcanbus",
        executable="qtcanbus_sender",
        name="qtcanbus_sender",
        parameters=[
            {"canbus_plugin": "peakcan", "canbus_interface": "usb0", "can_type": "sensor"}
        ],
        arguments=["--ros-args", "--log-level", "debug"]
    )

    qtcanbus_sender_motor = Node(
        package="ros2_qtcanbus",
        executable="qtcanbus_sender",
        name="qtcanbus_sender",
        parameters=[
            {"canbus_plugin": "peakcan", "canbus_interface": "usb1", "can_type": "motor"}
        ],
        arguments=["--ros-args", "--log-level", "debug"]
    )

    return LaunchDescription(
        [
            qtcanbus_sender_sensor,
            qtcanbus_sender_motor,
        ]
    )
