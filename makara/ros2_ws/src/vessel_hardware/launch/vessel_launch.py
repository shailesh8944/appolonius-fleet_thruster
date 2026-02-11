import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the launch argument
    vessel_id_arg = DeclareLaunchArgument(
        'vessel_id',
        default_value='0',
        description='ID of the vessel'
    )

    vessel_id = LaunchConfiguration('vessel_id')

    # Load the YAML file statically
    with open('/workspaces/mavlab/inputs/simulation_input.yml', 'r') as file:
        data = yaml.safe_load(file)

    # Create a static mapping from ID to name
    prefix_map = {}
    for idx, agent in enumerate(data['agents']):
        prefix_map[str(idx)] = agent['name']

    # Get the prefix for the current vessel_id
    topic_prefix = TextSubstitution(text=prefix_map.get(vessel_id, f"{prefix_map['0']}_00"))

    return LaunchDescription([
        vessel_id_arg,

        Node(
            package='uwb_driver',
            executable='uwb',
            name='uwb', 
            parameters=[{'topic_prefix': topic_prefix}]
        ),
        
        Node(
            package='vessel_hardware',
            executable='arduino',
            parameters=[{'topic_prefix': topic_prefix}]
        ),

        GroupAction([
            PushRosNamespace(topic_prefix),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("sbg_driver"), 
                        "launch", 
                        "sbg_device_launch.py"
                    ])
                ])
            )
        ])
    ])