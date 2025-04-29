import rclpy
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import Imu


class IMUDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("imu_mapping.canbus_message_acc_id", -1),
                ("imu_mapping.canbus_message_gyro_id", -1),
                ("imu_mapping.ros_frame_id", ""),
                ("imu_mapping.field_mapping.acc.x", ""),
                ("imu_mapping.field_mapping.acc.y", ""),
                ("imu_mapping.field_mapping.acc.z", ""),
                ("imu_mapping.field_mapping.acc.covariance", 0.),
                ("imu_mapping.field_mapping.gyro.x", ""),
                ("imu_mapping.field_mapping.gyro.y", ""),
                ("imu_mapping.field_mapping.gyro.z", ""),
                ("imu_mapping.field_mapping.gyro.covariance", 0.),
            ],
        )

        self.canbus_message_ids = {
            "acc": node.get_parameter("imu_mapping.canbus_message_acc_id").value,
            "gyro": node.get_parameter("imu_mapping.canbus_message_gyro_id").value
        }

        self.ros_frame_id = node.get_parameter("imu_mapping.ros_frame_id").value

        if -1 in self.canbus_message_ids.values():
            self.get_logger().warning(
                "No CAN frame ID specified for NavSatFix message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                f"GPSDecoder started with the following CAN frame ID: {self.canbus_message_ids}"
            )

        self.field_mapping = {}
        self.field_mapping["acc_x"] = node.get_parameter(
            "imu_mapping.field_mapping.acc.x"
        ).value
        self.field_mapping["acc_y"] = node.get_parameter(
            "imu_mapping.field_mapping.acc.y"
        ).value
        self.field_mapping["acc_z"] = node.get_parameter(
            "imu_mapping.field_mapping.acc.z"
        ).value
        self.field_mapping["acc_covariance"] = node.get_parameter(
            "imu_mapping.field_mapping.acc.covariance"
        ).value
        self.field_mapping["gyro_x"] = node.get_parameter(
            "imu_mapping.field_mapping.gyro.x"
        ).value
        self.field_mapping["gyro_y"] = node.get_parameter(
            "imu_mapping.field_mapping.gyro.y"
        ).value
        self.field_mapping["gyro_z"] = node.get_parameter(
            "imu_mapping.field_mapping.gyro.z"
        ).value
        self.field_mapping["gyro_covariance"] = node.get_parameter(
            "imu_mapping.field_mapping.gyro.covariance"
        ).value

        self.imu_pub = self.node.create_publisher(
            Imu,
            "/imu",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.acc_can_msg = None
        self.gyro_can_msg = None

    def is_imu_message(self, message) -> str:
        if message.id == self.canbus_message_ids["acc"]:
            return "acc"
        elif message.id == self.canbus_message_ids["gyro"]:
            return "gyro"
        else:
            return ""

    def decode_imu_message(self, canframe, msg, msg_type: str):
        self.get_logger().debug("Decoding IMU message: %s" % canframe)

        if msg_type == "acc":
            self.acc_can_msg = canframe
        elif msg_type == "gyro":
            self.gyro_can_msg = canframe

        if self.acc_can_msg is not None and self.gyro_can_msg is not None:

            imu_msg = Imu()
            imu_msg.header.frame_id = self.ros_frame_id
            imu_msg.header.stamp = msg.header.stamp

            if self.field_mapping["acc_x"]:
                imu_msg.linear_acceleration.x = self.acc_can_msg[self.field_mapping["acc_x"]]

            if self.field_mapping["acc_y"]:
                imu_msg.linear_acceleration.y = self.acc_can_msg[self.field_mapping["acc_y"]]

            if self.field_mapping["acc_z"]:
                imu_msg.linear_acceleration.z = self.acc_can_msg[self.field_mapping["acc_z"]]

            if self.field_mapping["gyro_x"]:
                imu_msg.angular_velocity.x = self.gyro_can_msg[self.field_mapping["gyro_x"]]

            if self.field_mapping["gyro_y"]:
                imu_msg.angular_velocity.y = self.gyro_can_msg[self.field_mapping["gyro_y"]]

            if self.field_mapping["gyro_z"]:
                imu_msg.angular_velocity.z = self.gyro_can_msg[self.field_mapping["gyro_z"]]

            covariance_acc = (float(self.field_mapping["acc_covariance"]), 0.0, 0.0,
                              0.0, float(self.field_mapping["acc_covariance"]), 0.0,
                              0.0, 0.0, float(self.field_mapping["acc_covariance"]))

            covariance_gyro = (self.field_mapping["gyro_covariance"], 0.0, 0.0,
                               0.0, self.field_mapping["gyro_covariance"], 0.0,
                               0.0, 0.0, self.field_mapping["gyro_covariance"])

            imu_msg.linear_acceleration_covariance = covariance_acc
            imu_msg.angular_velocity_covariance = covariance_gyro

            imu_msg.orientation_covariance = [-1., 0.0, 0.0,
                                            0.0, -1., 0.0,
                                            0.0, 0.0, -1.]

            self.imu_pub.publish(imu_msg)
            self.get_logger().info("Published IMU message")