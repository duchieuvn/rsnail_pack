from setuptools import find_packages, setup

package_name = 'odom_preprocessing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['launch/state_estimation_launch.py']),
        ('share/' + package_name, ['config/ekf.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='runningsnail',
    maintainer_email='flori-krebs@t-online.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["pose_calc = odom_preprocessing.pose_calc_node:main",
                            "twist_kistler_calc = odom_preprocessing.twist_calc_kistler:main"],
    },
)
