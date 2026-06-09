from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('dw_robot_base')

    config_file = os.path.join(
        package_share_dir,
        'config',
        'fake_diff_drive.yaml'
    )

    fake_diff_drive_node = Node(
        package='dw_robot_base',
        executable='fake_diff_drive_node',
        name='fake_diff_drive_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        fake_diff_drive_node
    ])
