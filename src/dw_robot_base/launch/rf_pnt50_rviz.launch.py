import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('dw_robot_base')

    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'rf_pnt50.rviz'
    )

    robot_urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'rf_pnt50_robot.urdf.xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                robot_urdf_file
            ])
        }]
    )

    odom_from_pnt50_feedback_node = Node(
        package='dw_robot_base',
        executable='odom_from_pnt50_feedback_node',
        name='odom_from_pnt50_feedback_node',
        output='screen',
        parameters=[{
            'feedback_topic': '/pnt50_feedback',
            'wheel_radius': 0.075,
            'wheel_separation': 0.40,
            'ticks_per_wheel_rev': 1000.0,
            'left_sign': 1.0,
            'right_sign': 1.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        odom_from_pnt50_feedback_node,
        rviz_node,
    ])