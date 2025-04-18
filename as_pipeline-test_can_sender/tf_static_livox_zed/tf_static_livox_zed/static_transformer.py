import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_broadcaster')

        # Declare parameters for the transform
        self.declare_parameter('parent_frame', 'zed_camera_center')
        self.declare_parameter('child_frame', 'livox_frame')
        self.declare_parameter('translation', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation', [0.0, 0.0, 0.0, 1.0])

        # Get parameters
        parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        translation = self.get_parameter('translation').get_parameter_value().double_array_value
        rotation = self.get_parameter('rotation').get_parameter_value().double_array_value

        # Initialize the broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create and publish the transform
        self.publish_static_transform(parent_frame, child_frame, translation, rotation)

    def publish_static_transform(self, parent_frame, child_frame, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(f'Broadcasting static transform from {parent_frame} to {child_frame}')

def main():
    rclpy.init()
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

