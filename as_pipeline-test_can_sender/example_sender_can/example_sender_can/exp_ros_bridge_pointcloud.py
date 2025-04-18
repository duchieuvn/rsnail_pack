#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class FrameBridge(Node):
    def __init__(self):
        super().__init__('frame_bridge')
        # Abonniere Nachrichten aus dem ersten Frame
        self.subscription = self.create_subscription(
            PointCloud2,
            'zed/zed_node/point_cloud/cloud_registered',  # Topic des ersten Frames
            self.callback,
            10)
        # Publisher für das Topic des zweiten Frames
        self.publisher = self.create_publisher(PointCloud2, 'camera/pointcloud', 10)
        self.get_logger().info("Frame Bridge Node gestartet.")

    def callback(self, msg:PointCloud2):
        # Erstelle eine neue Nachricht, in der nur das Frame geändert wird
        # Setze den neuen Frame (z.B. "second_frame")
        msg.header.frame_id = 'zed_frame'

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
