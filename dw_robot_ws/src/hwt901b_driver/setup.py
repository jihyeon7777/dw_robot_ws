from setuptools import find_packages, setup

package_name = 'hwt901b_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='jh@example.com',
    description='ROS2 driver for WIT Motion HWT901B-TTL IMU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hwt901b_node = hwt901b_driver.hwt901b_node:main',
        ],
    },
)