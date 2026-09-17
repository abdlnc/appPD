import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='AGV ROS2 control: motor (GPIO), autonomous, app-bridge, camera nodes',
    license='TODO',
    entry_points={
        'console_scripts': [
            'control_node = agv_control.control_node:main',
            'motor_node   = agv_control.motor_node:main',
            'bridge_node  = agv_control.bridge_node:main',
            'camera_node  = agv_control.camera_node:main',
        ],
    },
)
