import rclpy
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import Pose, Vector3, TwistWithCovarianceStamped
import numpy as np
from sympy.vector import Vector
from can_msgs.msg import Frame
from std_msgs.msg import Float32


class OpticalDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("vsu_mapping.canbus_message_vsu_distance_id", -1),
                ("vsu_mapping.canbus_message_vsu_angle_id", -1),
                ("vsu_mapping.ros_frame_id", ""),
                ("vsu_mapping.field_mapping.distance", ""),
                ("vsu_mapping.field_mapping.absolute_velocity", ""),
                ("vsu_mapping.field_mapping.angle", ""),
                ("vsu_mapping.field_mapping.vl", ""),
                ("vsu_mapping.field_mapping.vt", ""),
            ],
        )

        self.canbus_message_ids = {
            "distance": node.get_parameter("vsu_mapping.canbus_message_vsu_distance_id").value,
            "angle": node.get_parameter("vsu_mapping.canbus_message_vsu_angle_id").value
        }

        self.ros_frame_id = node.get_parameter("vsu_mapping.ros_frame_id").value

        if -1 in self.canbus_message_ids.values():
            self.get_logger().warning(
                "No CAN frame IDs specified for VSU message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                f"VSUDecoder started with the following CAN frame ID: {self.canbus_message_ids}"
            )

        self.field_mapping = {}
        self.field_mapping["dist"] = node.get_parameter(
            "vsu_mapping.field_mapping.distance"
        ).value
        self.field_mapping["absolute_velocity"] = node.get_parameter(
            "vsu_mapping.field_mapping.absolute_velocity"
        ).value
        self.field_mapping["angle"] = node.get_parameter(
            "vsu_mapping.field_mapping.angle"
        ).value
        self.field_mapping["vl"] = node.get_parameter(
            "vsu_mapping.field_mapping.vl"
        ).value
        self.field_mapping["vt"] = node.get_parameter(
            "vsu_mapping.field_mapping.vt"
        ).value

        self.angle_pub = self.node.create_publisher(
            Float32,
            "/vsu_angle",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.abs_velo_pub = self.node.create_publisher(
            Float32,
            "/vsu_absolute_velocity",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.velo_pub = self.node.create_publisher(
            TwistWithCovarianceStamped,
            "/vsu_velocity",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.distance_pub = self.node.create_publisher(
            Float32,
            "/vsu_distance",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.vsu_distance_can_msg = None
        self.vsu_angle_can_msg = None
        self.get_logger().debug("-----------------------------------------------VSU is up------------------------")
    def is_vsu_message(self, message) -> str:
        if message.id == self.canbus_message_ids["distance"]:
            return "distance_velo"
        elif message.id == self.canbus_message_ids["angle"]:
            return "angle_vel"
        else:
            return ""

    def decode_vsu_message(self, canframe, msg: Frame, msg_type: str):
        self.get_logger().debug(f"Decoding VSU message: {canframe}, msg_type {msg_type}, dist_field: -{self.field_mapping['dist']}-")

        if msg_type == "distance_velo":
            self.vsu_distance_can_msg = canframe

            distance_msg = Float32()
            velocity_msg = Float32()

            if self.field_mapping["dist"]:
                distance = self.vsu_distance_can_msg[self.field_mapping["dist"]]
                self.get_logger().debug(f"------Distance received: {distance}")
                distance_msg.data = distance
                self.distance_pub.publish(distance_msg)

            if self.field_mapping["absolute_velocity"]:
                velocity = self.vsu_distance_can_msg[self.field_mapping["absolute_velocity"]]
                velocity_msg.data = velocity
                self.abs_velo_pub.publish(velocity_msg)

        elif msg_type == "angle_vel":
            self.vsu_angle_can_msg = canframe

            angle_msg = Float32()
            velocity_msg = TwistWithCovarianceStamped()
            velocity_msg.header.frame_id = "kistler_frame"
            velocity_msg.header.stamp = msg.header.stamp


            if self.field_mapping["angle"]:
                angle = self.vsu_angle_can_msg[self.field_mapping["angle"]]
                velocity_long = self.vsu_angle_can_msg[self.field_mapping["vl"]]
                velocity_tran = self.vsu_angle_can_msg[self.field_mapping["vt"]]
                angle_msg.data = angle
                velocity_msg.twist.twist.linear.x = velocity_long
                velocity_msg.twist.twist.linear.y = velocity_tran

            velocity_msg.twist.covariance = np.eye(6).flatten() * 0.01

            self.angle_pub.publish(angle_msg)
            self.velo_pub.publish(velocity_msg)
