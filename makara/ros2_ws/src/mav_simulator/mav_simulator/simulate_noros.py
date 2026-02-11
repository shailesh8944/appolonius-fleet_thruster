"""
File: simulate_noros.py
Description: This file provides a standalone simulation environment for the MAV simulator
             without requiring ROS2. It:
             
             - Initializes the simulation environment from configuration files
             - Creates and configures vessel instances
             - Runs the simulation loop with time stepping
             - Displays simulation progress and results
             - Calculates and reports various forces acting on the vessel
             
             This module is useful for:
             - Quick testing of vessel dynamics without ROS2 overhead
             - Debugging core simulation functionality
             - Running batch simulations for parameter studies
             - Providing a simpler entry point for users unfamiliar with ROS2
             
Author: MAV Simulator Team
"""

from typing import NoReturn
import numpy as np
import sys
import os

# Add the parent directory to Python path to allow imports from this module 
sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

# Now import from the local module
from mav_simulator.read_input import read_input
from mav_simulator.class_vessel import Vessel

def simulate() -> NoReturn:
    """Run vessel simulation using parameters from input files."""
    # Read input files and create vessel
    sim_params, agents = read_input("/workspaces/mavlab/inputs/simulation_input.yml")
    vessel = Vessel(agents[0]['vessel_config'], agents[0]['hydrodynamics'], vessel_id=0, ros_flag=False)
    print("\nStarting simulation...")
    print("Time [s] | Position [x, y, z] | Velocity [u, v, w]")
    print("-" * 60)
    
    # Run simulation
    vessel.reset()
    while vessel.t < vessel.Tmax:
        # Step simulation
        vessel.step()
        
        # Extract position and velocity
        pos = vessel.current_state[6:9]  # [x, y, z]
        vel = vessel.current_state[0:3]  # [u, v, w]
        # Calculate forces from different components
        angles = vessel.current_state[9:12]  # [phi, theta, psi]
        F_hyd = vessel.hydrodynamic_forces(vessel.current_state[0:6])
        F_g = vessel.gravitational_forces(angles[0], angles[1])
        
        # Get control surface and thruster forces if they exist
        F_control = vessel.control_forces(vessel.current_state[12:]) if hasattr(vessel, 'control_surfaces') else np.zeros(6)
        F_thrust = vessel.thruster_forces(vessel.current_state[-len(vessel.thrusters['thrusters']):]) if hasattr(vessel, 'thrusters') else np.zeros(6)
        
        # Get control surface angles and thruster RPMs
        control_start = 12  # After position, velocity and angles
        thruster_start = control_start + len(vessel.control_surfaces['control_surfaces']) if hasattr(vessel, 'control_surfaces') else control_start
        control_angles = vessel.current_state[control_start:thruster_start] if hasattr(vessel, 'control_surfaces') else []
        thruster_rpms = vessel.current_state[thruster_start:] if hasattr(vessel, 'thrusters') else []
        
        # Print to terminal
        print(f"\nTime: {vessel.t:6.2f}")
        print(f"Position [x, y, z]: [{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}]")
        print(f"Velocity [u, v, w]: [{vel[0]:6.2f}, {vel[1]:6.2f}, {vel[2]:6.2f}]")
        print(f"Orientation [phi, theta, psi] (deg): [{angles[0]*180/np.pi:6.2f}, {angles[1]*180/np.pi:6.2f}, {angles[2]*180/np.pi:6.2f}]")
        print(f"Hydrodynamic forces: [{F_hyd[0]:6.2f}, {F_hyd[1]:6.2f}, {F_hyd[2]:6.2f}, {F_hyd[3]:6.2f}, {F_hyd[4]:6.2f}, {F_hyd[5]:6.2f}]")
        print(f"Gravitational forces: [{F_g[0]:6.2f}, {F_g[1]:6.2f}, {F_g[2]:6.2f}, {F_g[3]:6.2f}, {F_g[4]:6.2f}, {F_g[5]:6.2f}]")
        print(f"Control surface forces: [{F_control[0]:6.2f}, {F_control[1]:6.2f}, {F_control[2]:6.2f}, {F_control[3]:6.2f}, {F_control[4]:6.2f}, {F_control[5]:6.2f}]")
        print(f"Thruster forces: [{F_thrust[0]:6.2f}, {F_thrust[1]:6.2f}, {F_thrust[2]:6.2f}, {F_thrust[3]:6.2f}, {F_thrust[4]:6.2f}, {F_thrust[5]:6.2f}]")
        print(f"Control surface angles [deg]: {[f'{angle*180/np.pi:6.2f}' for angle in control_angles]}")
        print(f"Thruster RPMs: {[f'{rpm:6.2f}' for rpm in thruster_rpms]}")  # Convert RPS to RPM
    
    print("-" * 60)
    print("Simulation complete!")

if __name__ == "__main__":
    simulate()