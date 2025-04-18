import rclpy
from geometry_msgs.msg import Vector3, Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from std_msgs.msg import Float32


class TwistCalcNode(Node):
    def __init__(self):
        super().__init__("twist_calc_node")

        # subscribe to velocity and steering angle -> sync topics
        #self.subscription_lin_velocity = Subscriber(
        #    self,
        #    Vector3,
        #    "/vsu_absolute_velocity"
        #)
        #self.subscription_component_velocity = Subscriber(
        #    self,
        #    Vector3,
        #    "/vsu_velocity"
        #)
        self.subscription_angle = Subscriber(
            self,
            Float32,
            "/steering"
        )

        self.tss = ApproximateTimeSynchronizer(
            [self.subscription_lin_velocity,
             self.subscription_component_velocity,
             self.subscription_angle], 10, 0.1)
        self.tss.registerCallback(self.twist_calc_callback)

        self.publish_twist = self.create_publisher(
            Twist,
            "/twist",
            rclpy.qos.qos_profile_sensor_data
        )

    def twist_calc_callback(self, lin_velocity, component_velocity, angle):
        twist_msg = Twist()
        twist_msg.linear.x = lin_velocity.x + component_velocity.x
        twist_msg.linear.y = lin_velocity.y + component_velocity.y
        twist_msg.linear.z = lin_velocity.z + component_velocity.z

        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = angle.data

        self.publish_twist.publish(twist_msg)