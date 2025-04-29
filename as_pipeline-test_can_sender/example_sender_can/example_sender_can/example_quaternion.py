import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from scipy.spatial.transform import Rotation as R


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.get_quaternion)

    def get_quaternion(self, target_frame = 'zed_camera_link', source_frame = 'zed_imu_link'):
        try:
            # Wait and lookup the transform
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            # Access the quaternion from the rotation field
            quaternion = trans.transform.rotation
            translation = trans.transform.translation
            self.get_logger().info(
                f"Quaternion: x={quaternion.x}, y={quaternion.y}, z={quaternion.z}, w={quaternion.w}, {translation}")
            list_of_quaternion = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
            r = R.from_quat(list_of_quaternion)
            r_inv = r.inv()
            quaternion = r_inv.as_quat()

            self.get_logger().info(
                f"Inverted Quaternion: {quaternion} ")
            return quaternion
        except Exception as e:
            self.get_logger().error(f"Transform not available: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    # Replace 'target_frame' and 'source_frame' with your frame names
    #node.get_quaternion('base_link', 'livox_frame')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
