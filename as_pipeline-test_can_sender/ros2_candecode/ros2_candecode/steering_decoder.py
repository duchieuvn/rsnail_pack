import rclpy
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import numpy as np
from sympy.vector import Vector


class SteeringDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("steering_mapping.canbus_message_id", -1),
                ("steering_mapping.ros_frame_id", ""),
                ("steering_mapping.field_mapping.steering", ""),
            ],
        )

        self.canbus_message_ids = {
            "steering": node.get_parameter("steering_mapping.canbus_message_id").value
        }

        self.ros_frame_id = node.get_parameter("steering_mapping.ros_frame_id").value

        if -1 in self.canbus_message_ids.values():
            self.get_logger().warning(
                "No CAN frame IDs specified for Steering message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                f"SteeringDecoder started with the following CAN frame ID: {self.canbus_message_ids}"
            )

        self.field_mapping = {}
        self.field_mapping["steering"] = node.get_parameter(
            "steering_mapping.field_mapping.steering"
        ).value

        self.steering_pub = self.node.create_publisher(
            Float32,
            "/steering",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.steering_can_msg = None

    def is_steering_message(self, message) -> str:
        if message.id == self.canbus_message_ids["steering"]:
            return "steering"
        else:
            return ""

    def decode_steering_message(self, canframe, msg, msg_type: str):
        self.get_logger().debug("Decoding Steering message: %s" % canframe)

        if msg_type == "steering":
            self.steering_can_msg = canframe

            steering_msg = Float32()

            if self.field_mapping["steering"]:
                angle = self.steering_can_msg[self.field_mapping["steering"]]
                steering_msg.data = angle

            self.steering_pub.publish(steering_msg)
            self.get_logger().info("Published Steering message: %s" % steering_msg)