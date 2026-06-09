import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share_dir = get_package_share_directory('dw_robot_description')
    base_share_dir = get_package_share_directory('dw_robot_base')

    xacro_file = os.path.join(
        description_share_dir,
        'urdf',
        'dw_robot.urdf.xacro'
    )

    safety_config = os.path.join(
        base_share_dir,
        'config',
        'cmd_vel_safety.yaml'
    )

    fake_drive_config = os.path.join(
        base_share_dir,
        'config',
        'fake_diff_drive.yaml'
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            xacro_file
        ])
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    cmd_vel_safety_node = Node(
        package='dw_robot_base',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        parameters=[safety_config],
        output='screen'
    )

    fake_diff_drive_node = Node(
        package='dw_robot_base',
        executable='fake_diff_drive_node',
        name='fake_diff_drive_node',
        parameters=[fake_drive_config],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        cmd_vel_safety_node,
        fake_diff_drive_node,
        rviz_node
    ])