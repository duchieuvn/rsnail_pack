import rclpy
from rclpy.node import Node
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import ParameterDescriptor

import cantools

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from can_msgs.msg import Frame
from .imu_decoder import IMUDecoder
from .optical_decoder import OpticalDecoder
from .steering_decoder import SteeringDecoder
from .tire_decoder import TireDecoder


class CandecodeNode(Node):
    def __init__(self):
        super().__init__("candecode_node")
        self.imu_decoder = IMUDecoder(self)
        self.steering_decoder = SteeringDecoder(self)
        self.tire_decoder = TireDecoder(self)
        self.optical_decoder = OpticalDecoder(self)

        decode_choices_desc = ParameterDescriptor(
            description="Decode choices as strings"
        )
        warn_if_unknown_desc = ParameterDescriptor(
            description="Warn if unknown CAN ID received"
        )
        dbc_file_desc = ParameterDescriptor(
            description="Path to DBC file to use for decoding"
        )

        self.declare_parameter("decode_choices", False, decode_choices_desc)
        self.declare_parameter("warn_if_unknown", False, warn_if_unknown_desc)
        self.declare_parameter("dbc_file_S","signals.dbc",dbc_file_desc)
        self.declare_parameter("dbc_file_M","signals.dbc",dbc_file_desc)


        self.decode_choices = (
            self.get_parameter("decode_choices").get_parameter_value().bool_value
        )
        dbc_file_sensor = self.get_parameter("dbc_file_S").get_parameter_value().string_value
        dbc_file_motor = self.get_parameter("dbc_file_M").get_parameter_value().string_value
        self.warn_if_unknown = (
            self.get_parameter("warn_if_unknown").get_parameter_value().bool_value
        )

        self.db_sensor = cantools.database.load_file(dbc_file_sensor, strict=False)
        self.db_motor = cantools.database.load_file(dbc_file_motor, strict=False)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.subscription_sensor = self.create_subscription(
            Frame,
            "sensor_can",
            self.listener_callback_sensor,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.subscription_motor = self.create_subscription(
            Frame,
            "motor_can",
            self.listener_callback_motor,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.get_logger().info("Candecode initalized")

    def listener_callback_sensor(self, msg):
        self.get_logger().debug('received message sensor: "%s/%s"' % (msg.id, hex(msg.id)))

        try:
            val = self.db_sensor.decode_message(
                msg.id, msg.data.tobytes(order="C"), decode_choices=self.decode_choices
            )
            self.get_logger().debug(f"sensor decode: val: {val}, msg_id: {msg.id}, msg: {msg}")
            #self.publish_diagnostics(val, msg)
            msg_type = self.imu_decoder.is_imu_message(msg)
            if msg_type:
                self.imu_decoder.decode_imu_message(val, msg, msg_type)

            msg_type = self.steering_decoder.is_steering_message(msg)
            if msg_type:
                self.steering_decoder.decode_steering_message(val, msg, msg_type)

            msg_type = self.optical_decoder.is_vsu_message(msg)
            if msg_type:
                self.optical_decoder.decode_vsu_message(val, msg, msg_type)

        except KeyError:
            if self.warn_if_unknown:
                self.get_logger().warn("Unknown CAN ID: %s" % msg.id)
        except ValueError as err:
            if self.warn_if_unknown:
                self.get_logger().warn(f"Error during decoding for CAN ID: {msg.id} - {err}")
        except cantools.database.errors.DecodeError:
            self.get_logger().warn(
                "Failed to decode CAN ID: %s/%s" % (msg.id, hex(msg.id))
            )

    def listener_callback_motor(self, msg):
        self.get_logger().debug('received message motor: "%s/%s"' % (msg.id, hex(msg.id)))

        try:
            val = self.db_motor.decode_message(
                msg.id, msg.data.tobytes(order="C"), decode_choices=self.decode_choices
            )
            #self.publish_diagnostics(val, msg)
            msg_type = self.tire_decoder.is_tire_message(msg)
            if msg_type:
                self.tire_decoder.decode_tire_message(val, msg, msg_type)

        except KeyError:
            if self.warn_if_unknown:
                self.get_logger().warn("Unknown CAN ID: %s" % msg.id)
        except ValueError as err:
            if self.warn_if_unknown:
                self.get_logger().warn(f"Error during decoding for CAN ID: {msg.id} - {err}")
        except cantools.database.errors.DecodeError:
            self.get_logger().warn(
                "Failed to decode CAN ID: %s/%s" % (msg.id, hex(msg.id))
            )

    def publish_diagnostics(self, val, msg):
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.hardware_id = hex(msg.id)

        for key in val:
            key_value = KeyValue()
            key_value.key = key
            key_value.value = str(val[key])
            status_msg.values.append(key_value)

        status_msg.name = self.db.get_message_by_frame_id(msg.id).name

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = msg.header.stamp
        diag_msg.header.frame_id = msg.header.frame_id
        diag_msg.status.append(status_msg)

        self.get_logger().debug(str(val))

        self.diagnostics_pub.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CandecodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
