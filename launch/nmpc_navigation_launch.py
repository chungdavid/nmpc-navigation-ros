from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('nmpc_navigation'),
        'config',
        'config.yaml'
    )
    
    nmpc_navigation_node = Node(
        package = 'nmpc_navigation',
        executable = 'nmpc_navigation_node',
        name = 'nmpc_navigation_ros',
        output = "screen",
        emulate_tty = True,
        parameters = [config]
    )

    ld.add_action(nmpc_navigation_node)

    return ld
