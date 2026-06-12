from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os 
import yaml

def generate_launch_description():
    launch_desc = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lab1_pkg'),
        'config',
        'params.yaml'
    )

    config_dict = yaml.safe_load(open(config,'r'))

    talker_node = Node(
        package='lab1_pkg',
        executable='talker',
        name='talker_node',
        parameters=[config]
    )

    relay_node = Node(
        package='lab1_pkg',
        executable='relay',
        name='relay_node',
        
    )

    launch_desc.add_action(talker_node)
    launch_desc.add_action(relay_node)

    return launch_desc

