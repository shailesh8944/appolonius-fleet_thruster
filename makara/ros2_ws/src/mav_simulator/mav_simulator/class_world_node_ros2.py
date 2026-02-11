"""
File: class_world_node_ros2.py
Description: This file defines the World_Node class which serves as the ROS2 node wrapper
             for the World class in the MAV simulator. The World_Node class:
             
             - Inherits from the ROS2 Node class to integrate with the ROS2 ecosystem
             - Creates a timer that triggers the simulation step function at a specified rate
             - Provides the ROS2 context for the World class to operate within
             - Serves as the main interface between the ROS2 framework and the simulation
             
             This class is a minimal but essential component that enables the simulator
             to run within the ROS2 environment, allowing for distributed simulation
             and integration with other ROS2 nodes.
             
Author: MAV Simulator Team
"""

from rclpy.node import Node
from mav_simulator.class_world import World

class World_Node(Node):
    rate = None
    def __init__(self, world_rate=100, world_file=None):
        super().__init__('world')
        self.rate = world_rate
        self.world = World(world_file)
 