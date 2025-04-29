#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FrameBridge(Node):
    def __init__(self):
        super().__init__('frame_bridge')
        # Abonniere Nachrichten aus dem ersten Frame
        self.subscription = self.create_subscription(
            PoseStamped,
            'zed/zed_node/pose',  # Topic des ersten Frames
            self.callback,
            10)
        # Publisher für das Topic des zweiten Frames
        self.publisher = self.create_publisher(PoseStamped, 'camera/pose', 10)
        self.get_logger().info("Frame Bridge Node gestartet.")

    def callback(self, msg):
        # Erstelle eine neue Nachricht, in der nur das Frame geändert wird.
        new_msg = PoseStamped()
        new_msg.header = msg.header
        new_msg.pose = msg.pose

        # Setze den neuen Frame (z.B. "second_frame")
        new_msg.header.frame_id = 'zed_frame'

        self.publisher.publish(new_msg)
        self.get_logger().info(
            f"Nachricht von Frame '{msg.header.frame_id}' in '{new_msg.header.frame_id}' überführt.")


def main(args=None):
    rclpy.init(args=args)
    node = FrameBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
