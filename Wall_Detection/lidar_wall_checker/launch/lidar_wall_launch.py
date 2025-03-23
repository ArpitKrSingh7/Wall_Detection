from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_wall_checker',
            executable='lidar_threshold_node',
            name='lidar_threshold_node',
            output='screen'
        ),
        Node(
            package='lidar_wall_checker',
            executable='wall_alert_node',
            name='wall_alert_node',
            output='screen'
        ),
        Node(
            package='lidar_wall_checker',
            executable='beep_alert_node',   # <-- CHANGED HERE
            name='beep_alert_node',
            output='screen'
        ),
    ])
