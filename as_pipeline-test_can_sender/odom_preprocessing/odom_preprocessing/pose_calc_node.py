import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose


class PoseCalcNode(Node):
    def __init__(self):
        super().__init__("pose_calc_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subscribe to distance and steering angle -> sync topics
        self.subscription_distance = Subscriber(
            self,
            Float32,
            "/vsu_distance",
            qos_profile=qos_profile
        )
        self.subscription_angle = Subscriber(
            self,
            Float32,
            "/steering",
            qos_profile=qos_profile
        )
        self.tss = ApproximateTimeSynchronizer(
            [self.subscription_distance, self.subscription_angle], 10, 0.1)
        self.tss.registerCallback(self.pose_calc_callback)

        self.publish_pose = self.create_publisher(
            Pose,
            "/pose",
            rclpy.qos.qos_profile_sensor_data
        )

        self.last_distance = None
        self.pose_x = 0.
        self.pose_y = 0.

    def pose_calc_callback(self, distance, angle):
        if self.last_distance is None:
            self.last_distance = distance
            return
        self.get_logger().info(f"distance {distance} angle {angle}")
        distance_diff = distance.data - self.last_distance.data
        self.last_distance = distance

        self.pose_x += distance_diff * np.cos(angle.data)
        self.pose_y += distance_diff * np.sin(angle.data)

        pose_msg = Pose()
        pose_msg.position.x = self.pose_x
        pose_msg.position.y = self.pose_y
        pose_msg.position.z = 0

        pose_msg.orientation.w = np.cos(angle.data / 2)
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = np.sin(angle.data / 2)

        self.publish_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    node = PoseCalcNode()  # Instantiate the node class
    rclpy.spin(node)  # Keep the node alive and process callbacks

    # Shutdown after exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
