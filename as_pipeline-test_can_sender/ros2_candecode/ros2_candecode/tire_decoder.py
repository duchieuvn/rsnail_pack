import rclpy
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import Pose, Vector3
import numpy as np
from sympy.vector import Vector


class TireDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("tire_mapping.canbus_message_fl_id", -1),
                ("tire_mapping.canbus_message_fr_id", -1),
                ("tire_mapping.canbus_message_rl_id", -1),
                ("tire_mapping.canbus_message_rr_id", -1),
                ("tire_mapping.ros_frame_id", ""),
                ("tire_mapping.field_mapping.FL.rpm", ""),
                ("tire_mapping.field_mapping.FR.rpm", ""),
                ("tire_mapping.field_mapping.RL.rpm", ""),
                ("tire_mapping.field_mapping.RR.rpm", ""),
                ("tire_mapping.field_mapping.FL.torq", ""),
                ("tire_mapping.field_mapping.FR.torq", ""),
                ("tire_mapping.field_mapping.RL.torq", ""),
                ("tire_mapping.field_mapping.RR.torq", ""),
            ],
        )

        self.canbus_message_ids = {
            "FL": node.get_parameter("tire_mapping.canbus_message_fl_id").value,
            "FR": node.get_parameter("tire_mapping.canbus_message_fr_id").value,
            "RL": node.get_parameter("tire_mapping.canbus_message_rl_id").value,
            "RR": node.get_parameter("tire_mapping.canbus_message_rr_id").value
        }

        self.ros_frame_id = node.get_parameter("tire_mapping.ros_frame_id").value

        if -1 in self.canbus_message_ids.values():
            self.get_logger().warning(
                "No CAN frame IDs specified for Tire message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                f"TireDecoder started with the following CAN frame ID: {self.canbus_message_ids}"
            )

        self.field_mapping = {}
        self.field_mapping["FL_rpm"] = node.get_parameter(
            "tire_mapping.field_mapping.FL.rpm"
        ).value
        self.field_mapping["FR_rpm"] = node.get_parameter(
            "tire_mapping.field_mapping.FR.rpm"
        ).value
        self.field_mapping["RL_rpm"] = node.get_parameter(
            "tire_mapping.field_mapping.RL.rpm"
        ).value
        self.field_mapping["RR_rpm"] = node.get_parameter(
            "tire_mapping.field_mapping.RR.rpm"
        ).value

        self.field_mapping["FL_torq"] = node.get_parameter(
            "tire_mapping.field_mapping.FL.torq"
        ).value
        self.field_mapping["FR_torq"] = node.get_parameter(
            "tire_mapping.field_mapping.FR.torq"
        ).value
        self.field_mapping["RL_torq"] = node.get_parameter(
            "tire_mapping.field_mapping.RL.torq"
        ).value
        self.field_mapping["RR_torq"] = node.get_parameter(
            "tire_mapping.field_mapping.RR.torq"
        ).value


        self.fl_pub = self.node.create_publisher(
            Vector3,
            "/fl_tire",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.fr_pub = self.node.create_publisher(
            Vector3,
            "/fr_tire",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.rl_pub = self.node.create_publisher(
            Vector3,
            "/rl_tire",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.rr_pub = self.node.create_publisher(
            Vector3,
            "/rr_tire",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.tire_can_msg = None

    def is_tire_message(self, message) -> str:
        if message.id == self.canbus_message_ids["FL"]:
            return "FL"
        elif message.id == self.canbus_message_ids["FR"]:
            return "FR"
        elif message.id == self.canbus_message_ids["RL"]:
            return "RL"
        elif message.id == self.canbus_message_ids["RR"]:
            return "RR"
        else:
            return ""

    def decode_tire_message(self, canframe, msg, msg_type: str):
        self.get_logger().debug("Decoding Tire message: %s" % canframe)

        if msg_type == "FL":
            self.tire_can_msg = canframe

            tire_msg = Vector3()

            if self.field_mapping["FL_rpm"]:
                rpm = self.tire_can_msg[self.field_mapping["FL_rpm"]]
                tire_msg.x = rpm

            if self.field_mapping["FL_torq"]:
                torq = self.tire_can_msg[self.field_mapping["FL_torq"]]
                tire_msg.y = torq

            self.fl_pub.publish(tire_msg)

        elif msg_type == "FR":
            self.tire_can_msg = canframe

            tire_msg = Vector3()

            if self.field_mapping["FR_rpm"]:
                rpm = self.tire_can_msg[self.field_mapping["FR_rpm"]]
                tire_msg.x = rpm

            if self.field_mapping["FR_torq"]:
                torq = self.tire_can_msg[self.field_mapping["FR_torq"]]
                tire_msg.y = torq

            self.fr_pub.publish(tire_msg)

        elif msg_type == "RL":
            self.tire_can_msg = canframe

            tire_msg = Vector3()

            if self.field_mapping["RL_rpm"]:
                rpm = self.tire_can_msg[self.field_mapping["RL_rpm"]]
                tire_msg.x = rpm

            if self.field_mapping["RL_torq"]:
                torq = self.tire_can_msg[self.field_mapping["RL_torq"]]
                tire_msg.y = torq

            self.rl_pub.publish(tire_msg)

        elif msg_type == "RR":
            self.tire_can_msg = canframe

            tire_msg = Vector3()

            if self.field_mapping["RR_rpm"]:
                rpm = self.tire_can_msg[self.field_mapping["RR_rpm"]]
                tire_msg.x = rpm

            if self.field_mapping["RR_torq"]:
                torq = self.tire_can_msg[self.field_mapping["RR_torq"]]
                tire_msg.y = torq

            self.rr_pub.publish(tire_msg)
            self.get_logger().info(f"Publish CAN ID: {msg.id}")
