import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf',  # Replace with your actual package name
            executable='simple_ekf_node',  # Replace with your node executable name
            name='simple_ekf_node',
            output='screen'
        )
    ])


