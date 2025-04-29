from setuptools import find_packages, setup

package_name = 'example_sender_can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='as',
    maintainer_email='pfusch@runningsnail.oth-aw.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example_send_can = example_sender_can.exp_send_node:main",
            "example_quaternion = example_sender_can.example_quaternion:main",
            "example_ros_bridge_pose = example_sender_can.exp_ros_bridge_pose:main",
            "example_ros_bridge_pointcloud = example_sender_can.exp_ros_bridge_pointcloud:main",
            "example_ros_bridge_imu = example_sender_can.exp_ros_bridge_imu:main",
        ],
    },
)
