from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

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
            cmd=['python3', '-m', 'http.server', '8000', '--directory', web_build_dir],
            name='http_server',
            output='screen'
        ),
        # Start the rosbridge server
       # IncludeLaunchDescription(
        #    AnyLaunchDescriptionSource(rosbridge_launch)
       # ),        
    ])
