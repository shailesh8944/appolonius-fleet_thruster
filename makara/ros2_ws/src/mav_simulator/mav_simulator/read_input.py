"""
File: read_input.py
Description: This file provides utilities for reading and processing input configuration files
             for the MAV simulator. It includes functions for:
             
             - Loading and parsing YAML configuration files
             - Resolving file paths with vessel name substitution
             - Processing vessel and world configuration parameters
             - Handling hydrodynamic data inputs
             - Setting up initial conditions for simulation
             - Coordinating the loading of various input files needed for simulation
             
             This module serves as the configuration interface for the simulator,
             allowing users to define vessel properties, simulation parameters,
             and environmental conditions through external configuration files.
             
Author: MAV Simulator Team
"""

from typing import Dict, Tuple, List
import numpy as np
import yaml
import os
from pathlib import Path
import sys
from mav_simulator.generate_cross_flow import CrossFlowGenerator

def load_yaml(file_path: str) -> Dict:
    """Load a YAML file and return its contents.
    
    Args:
        file_path: Path to the YAML file
        
    Returns:
        Dict containing the YAML contents
    """
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def resolve_path(base_path: str, file_path: str, name: str) -> str:
    """Resolve a file path, replacing {name} with the actual vessel name.
    
    Args:
        base_path: Base directory path
        file_path: File path from YAML
        name: Vessel name to substitute
        
    Returns:
        Resolved absolute path
    """
    resolved = file_path.replace('{name}', name)
    if not os.path.isabs(resolved):
        resolved = os.path.join(base_path, resolved)

    # Fallback: if path does not exist (e.g., hardcoded /workspaces paths inside container),
    # try to map to a nearby inputs directory (installed share/ or source tree).
    if not os.path.exists(resolved):
        try:
            resolved_parts = Path(resolved).parts
            tail = None
            if 'inputs' in resolved_parts:
                tail = Path(*resolved_parts[resolved_parts.index('inputs') + 1:])
            if tail is not None:
                # 1) Installed share/mav_simulator/inputs
                pkg_inputs = Path(__file__).resolve().parents[2] / 'share' / 'mav_simulator' / 'inputs'
                alt = pkg_inputs / tail
                if alt.exists():
                    return str(alt)
                # 2) Walk up parents to find an inputs/ dir in the workspace
                for parent in Path(__file__).resolve().parents:
                    candidate = parent / 'inputs' / tail
                    if candidate.exists():
                        return str(candidate)
        except Exception:
            pass

    return resolved

def transform_to_co_frame(vessel_config: Dict) -> Dict:
    """Transform all component positions and orientations to be relative to the CO frame.
    
    Args:
        vessel_config: Vessel configuration dictionary
        
    Returns:
        Dict: Updated vessel configuration with transformed positions and orientations
    """
    # Extract CO position and orientation from geometry
    if 'geometry' not in vessel_config:
        return vessel_config  # No geometry data, return unchanged
    
    co_position = vessel_config['geometry'].get('CO', {}).get('position', [0.0, 0.0, 0.0])
    co_orientation = vessel_config['geometry'].get('CO', {}).get('orientation', [0.0, 0.0, 0.0])
    
    # Convert to numpy arrays for easier calculation
    co_position = np.array(co_position)
    co_orientation = np.array(co_orientation)
    
    # Create rotation matrices from the CO orientation
    # Using Three.js intrinsic Tait-Bryan angles in XYZ order
    def create_rotation_matrix(angles):
        # Rotation matrices for each axis
        x_angle, y_angle, z_angle = angles
        
        # Convert angles from degrees to radians if necessary
        x_angle_rad = np.radians(x_angle) if abs(x_angle) > np.pi * 2 else x_angle
        y_angle_rad = np.radians(y_angle) if abs(y_angle) > np.pi * 2 else y_angle
        z_angle_rad = np.radians(z_angle) if abs(z_angle) > np.pi * 2 else z_angle
        
        # X rotation matrix
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(x_angle_rad), -np.sin(x_angle_rad)],
            [0, np.sin(x_angle_rad), np.cos(x_angle_rad)]
        ])
        
        # Y rotation matrix
        Ry = np.array([
            [np.cos(y_angle_rad), 0, np.sin(y_angle_rad)],
            [0, 1, 0],
            [-np.sin(y_angle_rad), 0, np.cos(y_angle_rad)]
        ])
        
        # Z rotation matrix
        Rz = np.array([
            [np.cos(z_angle_rad), -np.sin(z_angle_rad), 0],
            [np.sin(z_angle_rad), np.cos(z_angle_rad), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix for intrinsic Tait-Bryan angles in XYZ order
        # R = Rz @ Ry @ Rx for intrinsic rotations
        R = Rx @ Ry @ Rz
        return R
    
    # Create inverse rotation matrices for transforming from CO frame
    co_rotation_matrix = create_rotation_matrix(co_orientation)
    co_rotation_matrix_inverse = co_rotation_matrix.T  # For rotation matrices, transpose = inverse
    
    # Function to transform a position from world frame to CO frame
    def transform_position(pos):
        if pos is None or pos == "None":
            return pos
        
        pos_array = np.array(pos)
        # Translate to CO frame
        transformed_pos = pos_array - co_position
        # Rotate to CO frame
        transformed_pos = co_rotation_matrix_inverse @ transformed_pos
        return transformed_pos.tolist()
    
    # Function to transform an orientation from world frame to CO frame
    def transform_orientation(orient):
        if orient is None or orient == "None":
            return orient
        
        # If orientation is a quaternion (4 elements)
        if isinstance(orient, list) and len(orient) == 4:
            # We would need quaternion operations for this
            # This is a placeholder - actual quaternion transformation would be required
            return orient
        
        # If orientation is Euler angles (3 elements)
        orient_array = np.array(orient)
        
        # For Tait-Bryan angles, the transformation is more complex
        # This is a simplified approach that may need refinement
        # We'd need to convert to rotation matrices, combine them, and convert back
        
        # Convert orientation to rotation matrix
        orient_matrix = create_rotation_matrix(orient_array)
        
        # Combined rotation relative to CO frame
        relative_rotation = co_rotation_matrix_inverse @ orient_matrix
        
        # Convert back to Euler angles
        # This is a simplified approach - a more robust method would use proper decomposition
        x_angle = np.arctan2(relative_rotation[2, 1], relative_rotation[2, 2])
        y_angle = np.arcsin(-relative_rotation[2, 0])
        z_angle = np.arctan2(relative_rotation[1, 0], relative_rotation[0, 0])
        
        # Convert to degrees if the original angles were in degrees
        if abs(orient_array[0]) > np.pi * 2 or abs(orient_array[1]) > np.pi * 2 or abs(orient_array[2]) > np.pi * 2:
            x_angle = np.degrees(x_angle)
            y_angle = np.degrees(y_angle)
            z_angle = np.degrees(z_angle)
            
        return [x_angle, y_angle, z_angle]
    
    # Transform geometrical points
    if 'geometry' in vessel_config:
        geometry = vessel_config['geometry']
        # Transform CG position and orientation relative to CO
        if 'CG' in geometry:
            if 'position' in geometry['CG']:
                geometry['CG']['position'] = transform_position(geometry['CG']['position'])
        
        # Transform CB position and orientation relative to CO
        if 'CB' in geometry:
            if 'position' in geometry['CB']:
                geometry['CB']['position'] = transform_position(geometry['CB']['position'])
            
        # Set CO to origin with zero orientation
        if 'CO' in geometry:
            geometry['CO']['position'] = [0.0, 0.0, 0.0]
            geometry['CO']['orientation'] = [0.0, 0.0, 0.0]
    
    # Transform thrusters
    if 'thrusters' in vessel_config and vessel_config['thrusters'] is not None:
        for thruster in vessel_config['thrusters'].get('thrusters', []):
            if 'thruster_location' in thruster:
                thruster['thruster_location'] = transform_position(thruster['thruster_location'])
            if 'thruster_orientation' in thruster:
                thruster['thruster_orientation'] = transform_orientation(thruster['thruster_orientation'])
    
    # Transform control surfaces
    if 'control_surfaces' in vessel_config and vessel_config['control_surfaces'] is not None:
        for surface in vessel_config['control_surfaces'].get('control_surfaces', []):
            if 'control_surface_location' in surface:
                surface['control_surface_location'] = transform_position(surface['control_surface_location'])
            if 'control_surface_orientation' in surface:
                surface['control_surface_orientation'] = transform_orientation(surface['control_surface_orientation'])
    
    # Transform sensors
    if 'sensors' in vessel_config and vessel_config['sensors'] is not None:
        for sensor in vessel_config['sensors'].get('sensors', []):
            if 'sensor_location' in sensor and sensor['sensor_location'] != 'None':
                sensor['sensor_location'] = transform_position(sensor['sensor_location'])
            if 'sensor_orientation' in sensor and sensor['sensor_orientation'] != 'None':
                sensor['sensor_orientation'] = transform_orientation(sensor['sensor_orientation'])
    
    return vessel_config

def read_input(input_file: str = None) -> Tuple[Dict, List[Dict]]:
    """Read vessel parameters and configuration from input files.
    
    Args:
        input_file: Path to the main simulation input file. If None, uses default.
        
    Returns:
        Tuple containing:
            sim_params (Dict): Global simulation parameters
            agents (List[Dict]): List of agents, each containing vessel config and hydrodynamics
    """    
    
    base_path = str(Path(input_file).parent.parent)
    sim_config = load_yaml(input_file)
    
    # Extract global simulation parameters
    sim_params = {
        'sim_time': sim_config.get('sim_time', 100),
        'time_step': sim_config.get('time_step', 0.01),
        'density': sim_config.get('density', 1000),
        'gravity': sim_config.get('gravity', 9.80665),
        'world_size': sim_config.get('world_size', [100, 100, float('inf')]),
        'gps_datum': sim_config.get('gps_datum', [0, 0, 0]),
        'nagents': sim_config.get('nagents', 1),
        'geofence': sim_config.get('geofence', []),
        'static_obstacle': sim_config.get('static_obstacle', [])
    }
    
    # Process each vessel's configuration
    agents = []
    
    for agent in sim_config.get('agents', []):
        vessel_name = agent['name']
        vessel_config = {
            'name': vessel_name,
            'type': agent['type'],
            'active_dof': agent['active_dof'],
            'U': agent['U'],
            'maintain_speed': agent['maintain_speed']
        }
        
        # Required for cross-flow drag generation
        gdf_file = load_yaml(resolve_path(base_path, agent['geometry'], vessel_name))['geometry_file']
        hydra_file = load_yaml(resolve_path(base_path, agent['hydrodynamics'], vessel_name))['hydra_file']
        hydrodynamics_file = resolve_path(base_path, agent['hydrodynamics'], vessel_name)
        initial_conditions_file = resolve_path(base_path, agent['initial_conditions'], vessel_name)
        cross_flow_generator = CrossFlowGenerator(gdf_file, hydra_file, hydrodynamics_file, initial_conditions_file, agent['type'])
        cross_flow_generator.update_yaml_file()
        
        # Load all referenced configuration files
        for config_type in ['geometry', 'inertia', 'hydrodynamics', 'control_surfaces', 
                          'initial_conditions', 'sensors', 'guidance', 'control', 'thrusters']:
            if config_type in agent:
                file_path = resolve_path(base_path, agent[config_type], vessel_name)
                vessel_config[config_type] = load_yaml(file_path)

        # Add simulation parameters to vessel config
        vessel_config.update({
            'sim_time': sim_params['sim_time'],
            'time_step': sim_params['time_step'],
            'density': sim_params['density'],
            'gravity': sim_params['gravity'],
            'gps_datum': sim_params['gps_datum'],
            'geofence': sim_params['geofence'],
            'static_obstacle': sim_params['static_obstacle']
        })
        
        # Transform all positions and orientations to CO frame
        vessel_config = transform_to_co_frame(vessel_config)
        
        # Create agent dictionary with vessel config and hydrodynamics
        agent_data = {
            'vessel_config': vessel_config,
            'hydrodynamics': vessel_config.get('hydrodynamics', {})
        }
        agents.append(agent_data)
            
    return sim_params, agents

if __name__ == "__main__":
    # Example usage
    input_file = input("Please enter the path to the input file: ")
    sim_params, agents = read_input(input_file)
    print("Simulation parameters:")
    print(sim_params)
    print("\nAgents:")
    for agent in agents:
        vessel_config = agent['vessel_config']
        print(f"\nVessel: {vessel_config['name']}")
        print(f"Initial position: {vessel_config.get('initial_position')}")
        print(f"Initial velocity: {vessel_config.get('initial_velocity')}")
        print(f"Control surfaces: {len(vessel_config.get('control_surfaces', []))}")
        print(f"Thrusters: {len(vessel_config.get('thrusters', []))}")
        print(f"Has hydrodynamics data: {bool(agent['hydrodynamics'])}")
        print(f"CO position: {vessel_config['geometry']['CO']['position']}")
        print(f"CO orientation: {vessel_config['geometry']['CO']['orientation']}")
        print(f"CG position: {vessel_config['geometry']['CG']['position']}")
        print(f"CG orientation: {vessel_config['geometry']['CG']['orientation']}")
        print(f"CB position: {vessel_config['geometry']['CB']['position']}")
        print(f"CB orientation: {vessel_config['geometry']['CB']['orientation']}")
        print(f"Thruster positions: {[t['thruster_location'] for t in vessel_config['thrusters']['thrusters']]}")
        print(f"Thruster orientations: {[t['thruster_orientation'] for t in vessel_config['thrusters']['thrusters']]}")
        print(f"Control surface positions: {[s['control_surface_location'] for s in vessel_config['control_surfaces']['control_surfaces']]}")
        print(f"Control surface orientations: {[s['control_surface_orientation'] for s in vessel_config['control_surfaces']['control_surfaces']]}")
