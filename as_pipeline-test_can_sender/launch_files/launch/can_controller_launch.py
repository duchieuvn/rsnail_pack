from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Node from example_sender_can
    can_controller = Node(
        package='example_sender_can',
        executable='example_send_can'
    )

    return LaunchDescription([
        can_controller
    ])
