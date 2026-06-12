from setuptools import setup
import os
from glob import glob

package_name = 'dw_robot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tsyim',
    maintainer_email='tsyim@example.com',
    description='DW robot base nodes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_diff_drive_node = dw_robot_base.fake_diff_drive_node:main',
            'cmd_vel_safety_node = dw_robot_base.cmd_vel_safety_node:main',
            'cmd_vel_mux_node = dw_robot_base.cmd_vel_mux_node:main',
            'rf_arduino_node = dw_robot_base.rf_arduino_node:main',
            'base_controller_node = dw_robot_base.base_controller_node:main',
            'pnt50_driver_node = dw_robot_base.pnt50_driver_node:main',
            'keyboard_brake_node = dw_robot_base.keyboard_brake_node:main',
            'usb_camera_node = dw_robot_base.usb_camera_node:main',
        ],
    },
)
