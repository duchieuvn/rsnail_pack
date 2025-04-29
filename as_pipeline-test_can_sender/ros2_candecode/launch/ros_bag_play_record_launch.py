import launch
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import datetime
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression


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
    file_name = None
    path = f"{os.path.expanduser('~')}/Desktop/ros_bags"
    if file_name == 'latest':
        subfolders = [f.name for f in os.scandir(path) if f.is_dir()]
        if subfolders:
            # Sort by file name in descending order assuming they follow the timestamp format 'YYYYMMDD_HHMMSS'
            sorted_subfolders = sorted(subfolders, reverse=True, key=lambda x: x)
            # Get the latest one (first item in the sorted list)
            latest_folder = sorted_subfolders[0]
        else:
            latest_folder = None
    else:
        latest_folder = file_name

    config = load_yaml('ros2_candecode', 'config/ros_bag_config.yaml')
    topic_groups = config.get('topic_groups', {})

    home_dir = str(os.path.expanduser("~"))

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'file_name',
                default_value='latest',
                description="directory of ros bags to be played"
            )
        ] + [
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', PythonExpression([f"'{home_dir}/Desktop/ros_bags/' + str(", LaunchConfiguration('file_name'), ")"])], #{bag_name}/{bag_name}_{group_name}_bag"],
                output='log'
            )
            for group_name, topics in topic_groups.items()
        ]

    )