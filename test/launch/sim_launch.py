from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution

import os

def generate_launch_description():

    package_name = 'test'

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'car-robot.rviz'  # Your RViz config file
    )

    # Path to the world file
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'simple_world.world'
    )

    # Path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'car-robot.xacro'   # Your new main xacro file
    )

    return LaunchDescription([

        # Launch Gazebo with the specified world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Publish the robot description to the /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),

        # Spawn the robot in Gazebo
        TimerAction(
            period=2.0,  # delay
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'skid_bot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
                    output='screen'
                ),
            ]
        ),

        # Optional: Launch teleop_twist_keyboard for manual control
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),

            Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # Comment/remove if no config file
        ),  
        
    ])


