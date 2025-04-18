import rclpy
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import TwistWithCovarianceStamped


class TwistDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("twist_mapping.canbus_message_id", -1),
                ("twist_mapping.frame_id", ""),
                ("twist_mapping.uom", "kph"),
                ("twist_mapping.field_mapping.linear.x", ""),
                ("twist_mapping.field_mapping.linear.y", ""),
                ("twist_mapping.field_mapping.linear.z", ""),
                ("twist_mapping.field_mapping.angular.x", ""),
                ("twist_mapping.field_mapping.angular.y", ""),
                ("twist_mapping.field_mapping.angular.z", ""),
                ("twist_mapping.covariance", 1e-2),
            ],
        )
        self.canbus_message_id = node.get_parameter(
            "twist_mapping.canbus_message_id"
        ).value
        self.frame_id = node.get_parameter("twist_mapping.frame_id").value

        if self.canbus_message_id == -1:
            self.get_logger().debug(
                "No CAN frame ID specified for Twist message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                "Twist started with the following CAN frame ID: %d"
                % self.canbus_message_id
            )

        self.field_mapping = {}
        self.field_mapping["linear_x"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.x"
        ).value
        self.field_mapping["linear_y"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.y"
        ).value
        self.field_mapping["linear_z"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.z"
        ).value
        self.field_mapping["angular_x"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.x"
        ).value
        self.field_mapping["angular_y"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.y"
        ).value
        self.field_mapping["angular_z"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.z"
        ).value

        self.uom = node.get_parameter("twist_mapping.uom").value
        self.covariance = node.get_parameter("twist_mapping.covariance").value

        self.twist_pub = self.node.create_publisher(
            TwistWithCovarianceStamped,
            "/vel",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.get_logger().info(
            "TwistDecoder started with the following message mappings: %s"
            % self.field_mapping
        )

    def is_twist_message(self, msg_id):
        return self.canbus_message_id == msg_id

    def decode_twist_message(self, canframe, msg):
        self.get_logger().debug("Decoding Twist message: %s" % canframe)

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.header.stamp = msg.header.stamp

        multiplier = 1.0
        if self.uom == "kph":
            multiplier = 1.0 / 3.6

        if self.field_mapping["linear_x"]:
            twist_msg.twist.twist.linear.x = (
                canframe.get(self.field_mapping["linear_x"]) * multiplier
            )

        if self.field_mapping["linear_y"]:
            twist_msg.twist.twist.linear.y = (
                canframe.get(self.field_mapping["linear_y"]) * multiplier
            )

        if self.field_mapping["linear_z"]:
            twist_msg.twist.twist.linear.z = (
                canframe.get(self.field_mapping["linear_z"]) * multiplier
            )

        if self.field_mapping["angular_x"]:
            twist_msg.twist.twist.angular.x = (
                canframe.get(self.field_mapping["angular_x"]) * multiplier
            )

        if self.field_mapping["angular_y"]:
            twist_msg.twist.twist.angular.y = (
                canframe.get(self.field_mapping["angular_y"]) * multiplier
            )

        if self.field_mapping["angular_z"]:
            twist_msg.twist.twist.angular.z = (
                canframe.get(self.field_mapping["angular_z"]) * multiplier
            )

        # fmt: off
        covariance = [self.covariance, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, self.covariance, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, self.covariance, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, self.covariance, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, self.covariance, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, self.covariance]
        # fmt: on

        twist_msg.twist.covariance = covariance

        self.twist_pub.publish(twist_msg)
