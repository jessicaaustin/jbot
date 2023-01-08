import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('jbot_autonomy'),
        'config',
        'jbot_autonomy_params.yaml'
    )

    node = Node(
        package='jbot_autonomy',
        name='jbot_autonomy',
        executable='jbot_autonomy',
        parameters=[config]
    )

    ld.add_action(node)
    return ld
