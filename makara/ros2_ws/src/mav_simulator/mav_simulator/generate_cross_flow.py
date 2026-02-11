"""
File: generate_cross_flow.py
Description: This file provides functionality for generating cross-flow hydrodynamic coefficients
             for marine vessels. The CrossFlowGenerator class:
             
             - Processes vessel geometry from GDF (Geometry Data File) files
             - Calculates hydrodynamic coefficients using strip theory
             - Handles different vessel types (AUV, surface vessels)
             - Updates hydrodynamic YAML configuration files with calculated coefficients
             - Integrates with the Grid class for geometry processing
             - Supports both command-line and programmatic usage
             
             This module is essential for accurately modeling the hydrodynamic forces
             on vessels, particularly for cross-flow effects that occur during non-zero
             sideslip angles or during turning maneuvers.
             
Author: MAV Simulator Team
"""

#!/usr/bin/env python3

import numpy as np
import yaml
import argparse
from pathlib import Path
from mav_simulator.class_grid import Grid
from mav_simulator.calculate_hydrodynamics import CalculateHydrodynamics
from mav_simulator.terminalMessages import print_info, print_warning, print_error

class CrossFlowGenerator:
    def __init__(self, gdf_file, hydra_file, yaml_file, initial_conditions_file, vessel_type='auv'):
        """Initialize the cross flow generator.
        
        Args:
            gdf_file (str): Path to the GDF file containing vessel geometry
            hydra_file (str): Path to the HydRA file containing hydrodynamic data
            yaml_file (str): Path to the hydrodynamics YAML file to update
            initial_conditions_file (str): Path to the initial conditions YAML file
            vessel_type (str): Type of vessel ('auv' or 'ship')
        """
        # Load initial conditions
        with open(initial_conditions_file, 'r') as f:
            init_cond = yaml.safe_load(f)
            
        start_location = np.array(init_cond['start_location'])
        start_orientation = np.array(init_cond['start_orientation'])
        
        # Initialize grid with initial conditions
        self.grid = Grid(gdf_file, start_location=start_location, start_orientation=start_orientation)
        self.grid.load_gdf()
        self.grid.create_2d_sections()
        
        self.hydra_file = hydra_file
        self.yaml_file = yaml_file
        self.length = self.grid.ulen
        self.U_des = 1.0  # Design speed in m/s
        self.vessel_type = vessel_type.lower()
        
        # Initialize hydrodynamics calculator
        self.hydro_calc = CalculateHydrodynamics()
        self.hydro_calc.grid = self.grid
        self.hydro_calc.length = self.length
        self.hydro_calc.U_des = self.U_des
        self.hydro_calc.ode_options = {}

    def calculate_coefficients(self):
        """Calculate cross-flow drag coefficients."""
        if self.vessel_type == 'auv':
            print_info("Calculating coefficients for AUV...")
            self.hydro_calc.cross_flow_drag_AUV()
        else:
            print_info("Calculating coefficients for surface ship...")
            self.hydro_calc.cross_flow_drag()
        return self.hydro_calc.ode_options

    def update_yaml_file(self):
        """Update the hydrodynamics YAML file with calculated coefficients.
        Only adds new coefficients, preserves existing ones. If cross_flow_drag
        is False, skips coefficient calculation entirely."""
        # Read existing YAML file
        with open(self.yaml_file, 'r') as f:
            hydro_data = yaml.safe_load(f)
        
        # Check if cross_flow_drag calculation is enabled
        cross_flow_enabled = hydro_data.get('cross_flow_drag', True)
        if not cross_flow_enabled:
            print_warning("cross_flow_drag is set to False. Skipping coefficient calculations.")
            return
            
        # Calculate new coefficients
        new_coeffs = self.calculate_coefficients()
        
        # Prepare comments for each coefficient group
        comments = {
            'X_': '\n# Surge hydrodynamic derivatives (X)',
            'Y_': '\n# Sway hydrodynamic derivatives (Y)',
            'Z_': '\n# Heave hydrodynamic derivatives (Z)',
            'N_': '\n# Yaw hydrodynamic derivatives (N)',
            'M_': '\n# Pitch hydrodynamic derivatives (M)'
        }
        
        # Create a new dictionary with comments
        output_str = "# Hydrodynamic coefficients calculated using strip theory and Hoerner's cross-flow drag\n"
        output_str += f"# New coefficients calculated for vessel type: {self.vessel_type.upper()}\n"
        output_str += "# All coefficients are in dimensional form\n"
        
        # Keep hydra_file and dim_flag at the top
        if 'hydra_file' in hydro_data:
            output_str += f"hydra_file: {hydro_data['hydra_file']}\n"
        if 'dim_flag' in hydro_data:
            output_str += f"dim_flag: {hydro_data['dim_flag']}\n"
        
        # Keep cross_flow_drag flag
        output_str += f"cross_flow_drag: {cross_flow_enabled}\n"
        
        # Group all coefficients by type
        coeff_groups = {'X_': [], 'Y_': [], 'Z_': [], 'N_': [], 'M_': []}
        
        # First add existing coefficients
        for key, value in hydro_data.items():
            if key not in ['hydra_file', 'dim_flag', 'cross_flow_drag']:
                for prefix in coeff_groups:
                    if key.startswith(prefix):
                        coeff_groups[prefix].append((key, value))
                        break
        
        # Then add new coefficients that don't exist yet
        for key, value in new_coeffs.items():
            exists = False
            for group in coeff_groups.values():
                if any(key == existing[0] for existing in group):
                    exists = True
                    break
            
            if not exists:
                for prefix in coeff_groups:
                    if key.startswith(prefix):
                        coeff_groups[prefix].append((key, float(value)))
                        print_info(f"Adding new coefficient: {key}")
                        break
        
        # Write each group with its comment
        for prefix, group in coeff_groups.items():
            if group:  # Only write groups that have coefficients
                output_str += comments[prefix] + '\n'
                for key, value in sorted(group):
                    output_str += f"{key}: {value}\n"
        
        # Write the output
        with open(self.yaml_file, 'w') as f:
            f.write(output_str)

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Generate cross-flow drag coefficients')
    parser.add_argument('--type', choices=['auv', 'ship'], default='auv',
                      help='Type of vessel (auv or ship)')
    args = parser.parse_args()
    
    # Get the base path
    base_path = Path('/workspaces/mavlab/inputs/mavymini')
    
    # Define file paths
    gdf_file = str(base_path / 'HydRA/input/MavyMini.gdf')
    hydra_file = str(base_path / 'HydRA/MAVYMINI_hydra.json')
    yaml_file = str(base_path / 'hydrodynamics.yml')
    initial_conditions_file = str(base_path / 'initial_conditions.yml')
    
    # Create generator and update coefficients
    generator = CrossFlowGenerator(gdf_file, hydra_file, yaml_file, initial_conditions_file, vessel_type=args.type)
    generator.update_yaml_file()
    print_info(f"Process completed for {args.type.upper()}")

if __name__ == '__main__':
    main() 