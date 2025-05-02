from setuptools import setup
import os
from glob import glob

package_name = 'robot_balancer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot balancer package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = robot_balancer.imu_node:main',
            'balance_controller_node = robot_balancer.balance_controller_node:main',
            'robot_hardware_interface = robot_balancer.robot_hardware_interface:main',
            'test_hardware = robot_balancer.test_hardware:main',
        ],
    },
) 