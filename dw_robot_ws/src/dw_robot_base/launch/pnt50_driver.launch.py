from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pnt50_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'pnt50_driver.yaml'
    ])

    pnt50_driver_node = Node(
        package='dw_robot_base',
        executable='pnt50_driver_node',
        name='pnt50_driver_node',
        output='screen',
        parameters=[pnt50_config_file]
    )

    return LaunchDescription([
        pnt50_driver_node,
    ])
