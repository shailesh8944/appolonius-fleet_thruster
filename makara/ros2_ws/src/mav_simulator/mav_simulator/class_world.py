"""
File: class_world.py
Description: This file defines the World class which serves as the simulation environment
             container for the MAV simulator. The World class:
             
             - Manages multiple vessel instances within a unified simulation environment
             - Handles initialization of the simulation world from configuration files
             - Coordinates the simulation time stepping across all vessels
             - Provides ROS2 integration for distributed simulation
             - Maintains global simulation parameters like GPS datum and world boundaries
             - Orchestrates the overall simulation execution
             
             The World class acts as the top-level controller for the entire simulation,
             bringing together vessels, environmental conditions, and simulation parameters.
             
Author: MAV Simulator Team
"""

#!/usr/bin/env python3

import numpy as np
import yaml
from mav_simulator.class_vessel import Vessel
from mav_simulator.class_vessel_pub_sub_ros2 import Vessel_Pub_Sub
from mav_simulator.read_input import read_input

class World():
    """
    A class representing the simulation world containing multiple vessels.

    Attributes:
        terminate (bool): Flag indicating whether the world simulation is terminated
        vessels (list): List of Vessel objects in the world
        nvessels (int): Number of vessels in the world
        size (numpy.ndarray): Size of the world in X-Y-Z dimensions
        gps_datum (numpy.ndarray): GPS reference point for the simulation
        node: ROS2 node reference
    """

    terminate = False                   # variable to note whether world is terminated
    vessels = []                        # list to store objects of class 'Vessel'
    nvessels = 0                        # number of vessels in the world
    size = np.zeros(3)                  # Initialization of Size of the world (X-Y-Z)
    gps_datum = None                    # GPS datum for the simulation (read from the inputs.yml file)
    node = None
    dt = 0.01


    def __init__(self, world_file=None):
        """
        Initialize the World object.

        Args:
            world_file (str, optional): Path to the YAML file containing world configuration.
                                      If provided, loads and processes the world data.
        """
        if world_file is not None:
            self.process_world_input(world_file)       # parse the world data dictionary to create dictionaries

    def start_vessel_ros_nodes(self,world_node):
        """
        Initialize ROS2 nodes for all vessels in the world.
        Creates a Vessel_Pub_Sub node for each vessel with their respective IDs.
        """
        for vessel in self.vessels:
            vessel.vessel_node = Vessel_Pub_Sub(vessel,world_node)
   
    def process_world_input(self, world_file=None):
        """
        Process the world configuration data from the YAML file.

        Args:
            data (dict, optional): Dictionary containing world configuration parameters including:
                - world_size: X-Y-Z dimensions of the world
                - time_step: Simulation time step
                - nagents: Number of vessels/agents
                - gps_datum: GPS reference coordinates
                - agents: List of vessel configurations
                - density: Optional fluid density
                - gravity: Optional gravity value

        Raises:
            yaml.YAMLError: If there's an error in parsing the YAML data
        """
        try:
            sim_params, agents = read_input(world_file)
            self.size = np.array(sim_params['world_size'])
            self.nvessels = sim_params['nagents']
            self.gps_datum = np.array(sim_params['gps_datum'])
            agent_count = 0
            self.dt = sim_params['time_step']
            
            for agent in agents[0:self.nvessels]:
                vessel_config = agent['vessel_config']
                hydrodynamic_data = agent['hydrodynamics']
                self.vessels.append(Vessel(vessel_params=vessel_config, hydrodynamic_data=hydrodynamic_data, vessel_id=agent_count))
                agent_count += 1

        except yaml.YAMLError as exc:
            print(exc)
            exit()

    def step(self):
        """
        Advance the simulation by one time step.
        Updates the state of all vessels and increments the current simulation time.
        """
        for vessel in self.vessels:
            vessel.step()