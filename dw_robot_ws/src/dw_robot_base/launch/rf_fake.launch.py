from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_description'),
        'urdf',
        'dw_robot.urdf.xacro'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_description'),
        'rviz',
        'dw_robot.rviz'
    ])

    rf_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'rf_arduino.yaml'
    ])

    mux_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'cmd_vel_mux.yaml'
    ])

    safety_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'cmd_vel_safety.yaml'
    ])

    fake_diff_drive_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'fake_diff_drive.yaml'
    ])

    robot_description = {
        'robot_description': Command([
            'xacro ',
            urdf_file
        ])
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rf_arduino_node = Node(
        package='dw_robot_base',
        executable='rf_arduino_node',
        name='rf_arduino_node',
        output='screen',
        parameters=[rf_config_file]
    )

    cmd_vel_mux_node = Node(
        package='dw_robot_base',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen',
        parameters=[
            mux_config_file,
            {'input_source': 'rf'}
        ]
    )

    cmd_vel_safety_node = Node(
        package='dw_robot_base',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        output='screen',
        parameters=[safety_config_file]
    )

    fake_diff_drive_node = Node(
        package='dw_robot_base',
        executable='fake_diff_drive_node',
        name='fake_diff_drive_node',
        output='screen',
        parameters=[fake_diff_drive_config_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rf_arduino_node,
        cmd_vel_mux_node,
        cmd_vel_safety_node,
        fake_diff_drive_node,
        rviz_node,
    ])