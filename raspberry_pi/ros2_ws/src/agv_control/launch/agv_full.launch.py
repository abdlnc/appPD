"""
AGV FULL bring-up: ONE command starts the whole system.

    sllidar_node  (Lidar driver)  -> /scan
    motor_node    -> owns GPIO, executes /cmd_vel
    control_node  -> autonomous logic, /scan -> /cmd_vel (while mode AUTO)
    bridge_node   -> WebSocket <-> app: /cmd_vel (manual) + /mode + camera relay
    camera_node   -> /image_raw/compressed + snapshot capture for the AI

Usage:
    ros2 launch agv_control agv_full.launch.py

(agv_bringup.launch.py still exists for control-only testing without the Lidar.)
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch', 'sllidar_c1_launch.py')

    return LaunchDescription([
        # Lidar driver (publishes /scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch)),

        # AGV nodes
        Node(package='agv_control', executable='motor_node',
             name='motor_node', output='screen'),
        Node(package='agv_control', executable='control_node',
             name='control_node', output='screen'),
        Node(package='agv_control', executable='bridge_node',
             name='bridge_node', output='screen'),
        Node(package='agv_control', executable='camera_node',
             name='camera_node', output='screen'),
    ])
