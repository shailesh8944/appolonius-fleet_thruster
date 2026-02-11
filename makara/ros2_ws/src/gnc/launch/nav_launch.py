import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import  PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mav_simulator.class_world import World

def generate_launch_description():

    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])
    
    # Path to the web directory in gnc package
    web_dir = PathJoinSubstitution([
        FindPackageShare('gnc'),
        'web'
    ])      
    
    return LaunchDescription([
        Node(
            package='gnc',
            executable='nav',
            name='nav',
            output='screen'
        ),
        # Start the HTTP server to serve the web interface
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000', '--directory', web_dir],
            name='http_server',
        ),
        # Start the rosbridge server
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch)
        ),
    ])
