from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # Path to the rosbridge launch file
    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])

    # Path to the built web directory
    web_build_dir = PathJoinSubstitution([
        FindPackageShare('mav_imu_visualizer'),
        'web'
    ])

    return LaunchDescription([
        # Start the HTTP server to serve web files
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8008', '--directory', web_build_dir],
            name='http_server',
            output='screen'
        ),
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
        ),
        Node(
            package='mav_imu_simulator',
            executable='mav_imu_simulator',
            name='mav_imu_simulator',
            output='screen'
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch)
        )
    ])
