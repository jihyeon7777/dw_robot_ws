from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# RF 수동 조종 + 4륜 메카넘 드라이브트레인.
#
# rf_pnt50.launch.py(차동)와 거의 동일하지만 base_controller와 pnt50_driver를
# drive_mode='mecanum'으로 띄운다. RF 입력 매핑은 rf_arduino_node가 담당하며,
# 아두이노 CSV에 CH7이 없으면 default_drive_mode(manual: 전후+yaw)로 동작한다.
# CH7이 들어오면 MANUAL/MECANUM(좌우 평행이동)/AUTO 가 자동 전환된다.
def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_description'),
        'urdf',
        'dw_robot.urdf.xacro'
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

    base_controller_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'base_controller.yaml'
    ])

    pnt50_config_file = PathJoinSubstitution([
        FindPackageShare('dw_robot_base'),
        'config',
        'pnt50_driver.yaml'
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
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

    base_controller_node = Node(
        package='dw_robot_base',
        executable='base_controller_node',
        name='base_controller_node',
        output='screen',
        parameters=[
            base_controller_config_file,
            {'drive_mode': 'mecanum'}
        ]
    )

    pnt50_driver_node = Node(
        package='dw_robot_base',
        executable='pnt50_driver_node',
        name='pnt50_driver_node',
        output='screen',
        parameters=[
            pnt50_config_file,
            {'drive_mode': 'mecanum'}
        ]
    )

    usb_camera_node = Node(
        package='dw_robot_base',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[
            {
                'camera_index': 0,
                'frame_id': 'usb_camera_optical_frame',
                'image_topic': '/usb_camera/image_raw',
                'width': 320,
                'height': 240,
                'fps': 5.0,
            }
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rf_arduino_node,
        cmd_vel_mux_node,
        cmd_vel_safety_node,
        base_controller_node,
        pnt50_driver_node,
        usb_camera_node,
    ])
