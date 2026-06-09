import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('dw_robot_base')

    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'rf_pnt50.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        rviz_node,
    ])