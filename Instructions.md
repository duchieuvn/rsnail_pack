## Steps to follow

git clone --branch 5.15 https://github.com/qt/qtserialbus.git

git clone --branch 4.2.0-ros https://github.com/borglab/gtsam.git

1. colcon build the packages you want to run
in order to launch the node, you have to build the specific node package,

right under workspace:

for example: Perception_filter
colcon build --packages-select perception_filter
colcon build --packages-select lidar_pipeline

2. source

source install/setup.bash

3. Launch the node (check the launch file you wanna to launch, which node is included, then you need to build each package accordingly)

inside the launch file you will see the node included, which you will launch, 


example: launch the perception_launch.py file

ros2 launch launch_files/launch/perception_launch.py

4. start Rviz2 (visualization tool)

run: rivz2

5. play the ros bag to show recorded data

ros2 bag play livox_20250307_1643_bag

