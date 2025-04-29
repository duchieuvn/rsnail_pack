from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    velodyne_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        #parameters=[
        #    {"x": 1.37, "y": 0.025, "z": 0.9, "yaw": 0.7679, "pitch": 0.1224, "roll": 0.0672, "frame-id": "base_link", "child-frame-id": "velodyne"},
        #],
        arguments=["--x", "1.37", "--y", "0.025", "--z", "0.9", "--yaw", "0.7679", "--pitch", "0.1224", "--roll", "0.0672", "--frame-id", "base_link", "--child-frame-id", "velodyne"]
    )

    velodyne_tf_point_cloud = Node(
        package="pointcloud_transformer",
        executable="pointcloud_transformer_node",
        name="pointcloud_transformer_node",
        parameters=[
            {"input_topic": "velodyne_points"}
        ],
        arguments=["--ros-args", "--log-level", "debug"]
    )

    livox_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        #parameters=[
        #    {"x": -0.166, "y": 0, "z": 0.9, "yaw": 0, "pitch": 0.0349, "roll": 0, "frame_id": "base_link", "child_frame_id": "livox_frame"},
        #],
        arguments=["--x", "-0.166", "--y", "0", "--z", "0.9", "--yaw", "0", "--pitch", "0.0349", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "livox_frame"]
    )

    livox_tf_point_cloud = Node(
        package="pointcloud_transformer",
        executable="pointcloud_transformer_node",
        name="pointcloud_transformer_node",
        parameters=[
            {"input_topic": "livox/lidar"}
        ],
        arguments=["--ros-args", "--log-level", "debug"]
    )

    # TODO anpassen -> nicht genau gemessen
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        #parameters=[
        #    {"x": -0.166, "y": 0, "z": 0.9, "yaw": 0, "pitch": 0.0349, "roll": 0, "frame_id": "base_link", "child_frame_id": "livox_frame"},
        #], 0.          0.01744911  0.         -0.99984775
        arguments=["--x", "-0.166", "--y", "0", "--z", "0.9", "--yaw", "0", "--pitch", "0.0349", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "zed_frame"]
    )

    # TODO anpassen -> nicht genau gemessen
    kistler_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        # parameters=[
        #    {"x": -0.166, "y": 0, "z": 0.9, "yaw": 0, "pitch": 0.0349, "roll": 0, "frame_id": "base_link", "child_frame_id": "livox_frame"},
        # ], 0.          0.01744911  0.         -0.99984775
        arguments=["--x", "-1.32", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "base_link", "--child-frame-id", "kistler_frame"]
    )

    camera_bridge_pose = Node(
        package="example_sender_can",
        executable="example_ros_bridge_pose",
        name="example_ros_bridge_pose",
    )

    camera_bridge_imu = Node(
        package="example_sender_can",
        executable="example_ros_bridge_imu",
        name="example_ros_bridge_imu",
    )

    camera_bridge_pointcloud = Node(
        package="example_sender_can",
        executable="example_ros_bridge_pointcloud",
        name="example_ros_bridge_pointcloud",
    )

    return LaunchDescription([
        velodyne_tf,
        velodyne_tf_point_cloud,
        livox_tf,
        livox_tf_point_cloud,
        camera_tf,
        kistler_tf,
        camera_bridge_pointcloud,
        camera_bridge_imu,
        camera_bridge_pose,
    ])