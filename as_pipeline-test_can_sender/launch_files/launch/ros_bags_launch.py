import launch
from launch.actions import ExecuteProcess
import datetime
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML configuration file."""
    package_share_directory = get_package_share_directory(package_name)
    yaml_file_path = os.path.join(package_share_directory, file_path)
    with open(yaml_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_bag_name():
    # Get current date and time
    now = datetime.datetime.now()
    return now.strftime("%Y%m%d_%H%M%S")


def generate_launch_description():
    config = load_yaml('ros2_candecode', 'config/ros_bag_config.yaml')
    topic_groups = config.get('topic_groups', {})

    # Create a list to hold all ExecuteProcess actions for rosbag recording
    rosbag_processes = []

    for group_name, topics in topic_groups.items():
        bag_name = generate_bag_name()
        bag_file_path = f"{os.path.expanduser('~')}/Desktop/ros_bags/{bag_name}/{group_name}_{bag_name}_bag"

        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_file_path] + topics,
            output='log'
        )
        rosbag_processes.append(rosbag_process)

    return launch.LaunchDescription(rosbag_processes)
