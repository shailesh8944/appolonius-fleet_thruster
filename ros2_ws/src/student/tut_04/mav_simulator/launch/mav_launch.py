from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mav_simulator',
            executable='mav_simulate',
            name='mav_simulate',
            output='screen'
        ),
        Node(
            package='mav_simulator',
            executable='mav_control',
            name='mav_control',
            output='screen'
        )
    ])