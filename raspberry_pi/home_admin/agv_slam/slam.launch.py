from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lidar ~gitna ng robot; pataas 0.1m. (Pwede i-adjust mamaya.)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0','0','0.1','0','0','0','base_link','laser'],
        ),
        # Optional: robot center ~sa lupa
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0','0','0','0','0','0','odom','base_link'],
        ),
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=['/home/admin/agv_slam/mapper.yaml'],
        ),
    ])
