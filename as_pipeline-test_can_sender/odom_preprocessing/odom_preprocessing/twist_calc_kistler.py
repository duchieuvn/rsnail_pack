import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist


class TwistKistlerNode(Node):
    def __init__(self):
        super().__init__("twist_kistler_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # subscribe to distance and steering angle -> sync topics
        self.subscription_velocity = Subscriber(
            self,
            Float32,
            "/vsu_absolute_velocity",
            qos_profile=qos_profile
        )
        self.subscription_imu = Subscriber(
            self,
            Twist,
            "/imu",
            qos_profile=qos_profile
        )
        self.tss = ApproximateTimeSynchronizer(
            [self.subscription_velocity, self.subscription_imu], 10, 0.1)
        self.tss.registerCallback(self.twist_calc_callback)

        self.publish_pose = self.create_publisher(
            Twist,
            "/twist_kistler",
            rclpy.qos.qos_profile_sensor_data
        )

    def twist_calc_callback(self, velocity, imu):
        # calculate twist
        twist_msg = Twist()
        twist_msg.linear.x = velocity.x
        twist_msg.linear.y = velocity.y
        twist_msg.linear.z = 0

        imu_ay = imu.linear_acceleration.y
        imu_vz = imu.angular_velocity.z

        imu_r = imu_ay/(imu_vz**2)
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = velocity.data/imu_r
        
        self.publish_pose.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    node = TwistKistlerNode()  # Instantiate the node class
    rclpy.spin(node)  # Keep the node alive and process callbacks

    # Shutdown after exit
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
