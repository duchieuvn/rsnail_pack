import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations  # For quaternion calculations

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Create a TransformBroadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Create a TransformStamped message
        transform = TransformStamped()

        # Set header info
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "/velodyne"
        transform.child_frame_id = "/velodyne_corr"

        # Define translation (no movement)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = -85.0

        # Define rotation (quaternion)
        # Example: +45 degrees Z, -5 degrees Y
        #quaternion = tf_transformations.quaternion_from_euler(0.0, -0.0873, 0.7854)  # Roll, Pitch, Yaw
        quaternion = [-0.02556583319918316, -0.06490264865201102, -0.3656084481562841, 0.9281511175490438]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(transform)
        self.get_logger().info('Published static transform.')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

