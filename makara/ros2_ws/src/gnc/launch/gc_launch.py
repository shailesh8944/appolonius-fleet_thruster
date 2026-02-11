import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from mav_simulator.class_world import World
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='gnc',
            executable='gc',
            name='gc',
            output='screen'
        ),
    ])