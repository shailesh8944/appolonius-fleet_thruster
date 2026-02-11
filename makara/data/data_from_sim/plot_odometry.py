#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import yaml
import glob
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def process_bag(bag_dir_path):
    """Process a single bag directory and return trajectory points"""
    # Set up bag reader
    storage_options = StorageOptions(
        uri=bag_dir_path,
        storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    # Initialize bag reader
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Store trajectory points
    x_vals = []
    y_vals = []
    
    # Process all messages
    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name == '/sookshma_00/odometry_sim':
            msg = deserialize_message(data, Odometry)
            x_vals.append(msg.pose.pose.position.x)
            y_vals.append(msg.pose.pose.position.y)
    
    return x_vals, y_vals

def main():
    # Initialize ROS context
    rclpy.init()
    
    # Get run directories from command line arguments or use default
    run_dirs = sys.argv[1:] if len(sys.argv) > 1 else []
    if not run_dirs:
        print("Usage: ./plot_odometry.py <path_to_run_dir1> [<path_to_run_dir2> ...]")
        print("Example: ./plot_odometry.py run1 ../data_from_sim/run2") # Adjusted example
        # Try to find run directories automatically in the script's directory
        script_dir = os.path.dirname(__file__)
        # Look for directories starting with 'run' in the script's directory
        potential_runs = glob.glob(os.path.join(script_dir, 'run*')) 
        # Filter out non-directories just in case glob returns files
        potential_runs = [p for p in potential_runs if os.path.isdir(p)] 

        if potential_runs:
            print(f"No run directories specified. Found and using automatically detected runs in {script_dir}: {potential_runs}")
            run_dirs = sorted(potential_runs)
        else:
            print(f"Error: No run directories specified and none found automatically in {script_dir}")
            rclpy.shutdown() # Shutdown ROS before exiting
            sys.exit(1)

    print(f"Processing run directories: {run_dirs}")
    
    # Process each bag and plot in its own subplot
    for i, run_dir in enumerate(run_dirs):
        print(f"\nProcessing run directory: {run_dir}")

        # --- Find Waypoints File ---
        waypoints_file = os.path.join(run_dir, 'waypoints.yaml')
        if not os.path.exists(waypoints_file):
            print(f" Error: Cannot find waypoints file: {waypoints_file}")
            continue

        # --- Load Waypoints ---
        try:
            with open(waypoints_file, 'r') as f:
                waypoints_data = yaml.safe_load(f)
            waypoints = np.array(waypoints_data['waypoints'])
            print(f" Loaded {len(waypoints)} waypoints from {waypoints_file}")
        except Exception as e:
            print(f" Error loading or parsing waypoints file {waypoints_file}: {e}")
            continue

        # --- Find Bag Directory ---
        # Expecting a directory like sim_data_runX_0 inside the run_dir
        # The pattern should match the directory, not the file inside it.
        bag_dir_pattern = os.path.join(run_dir, 'sim_data_run*') 
        potential_bag_dirs = glob.glob(bag_dir_pattern)
        # Filter results to ensure we only consider directories
        potential_bag_dirs = [p for p in potential_bag_dirs if os.path.isdir(p)]

        if not potential_bag_dirs:
            print(f" Error: Cannot find bag directory matching pattern: {bag_dir_pattern}")
            continue

        # Sort and pick the first one (usually _0)
        potential_bag_dirs.sort()
        bag_dir_path = potential_bag_dirs[0]
        print(f" Using bag directory: {bag_dir_path}")

        # Get trajectory data
        try:
            x_vals, y_vals = process_bag(bag_dir_path)
        except Exception as e:
            print(f" Error processing bag directory {bag_dir_path}: {e}")
            continue # Skip this run if error occurs

        # Create a new figure for each run
        fig = plt.figure(figsize=(8, 7))
        ax = plt.gca() # Get current axes for this figure
        
        # Plot trajectory
        ax.plot(x_vals, y_vals, label='Ship Trajectory', linewidth=2)
        
        # Plot waypoints
        ax.scatter(waypoints[:,0], waypoints[:,1], 
                  color='red', marker='*', s=100, label='Waypoints')
        
        # Connect waypoints with dashed lines
        ax.plot(waypoints[:,0], waypoints[:,1], 
                'k--', linewidth=1, label='Waypoint Path')
        
        # Add waypoint labels
        for wp in waypoints:
            x, y, _ = wp
            ax.text(x + 0.3, y + 0.3, f'({x:.1f}, {y:.1f})', 
                    fontsize=8, ha='left', va='bottom')
        
        # Set axis labels and title
        run_name = os.path.basename(run_dir.rstrip('/'))
        ax.set_title(f"Trajectory for Run: {run_name}")
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.grid(True)
        ax.legend()
        ax.axis('equal')
        
        # Invert Y-axis for top-left origin
        ax.invert_yaxis()  # Y increases downward
        
        # Adjust layout for the current figure
        plt.tight_layout()

        # Save the individual plot inside the run directory
        save_filename = f"{run_name}_trajectory.png"
        save_path = os.path.join(run_dir, save_filename)
        try:
            plt.savefig(save_path, bbox_inches='tight')
            print(f"Plot saved as {save_path}")
        except Exception as e:
            print(f" Error saving plot {save_path}: {e}")

        # Close the figure to free memory
        plt.close(fig)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()