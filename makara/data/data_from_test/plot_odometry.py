#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def process_bag(bag_path):
    """Process a single bag file and return trajectory points"""
    # Set up bag reader
    storage_options = StorageOptions(
        uri=bag_path,
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
        if topic_name == '/sookshma_00/odometry':
            msg = deserialize_message(data, Odometry)
            x_vals.append(msg.pose.pose.position.x)
            y_vals.append(msg.pose.pose.position.y)
    
    return x_vals, y_vals

def main():
    # Initialize ROS context
    rclpy.init()
    
    # Get bag paths from command line arguments or use default
    bag_paths = sys.argv[1:] if len(sys.argv) > 1 else ['./data_from_test/mavtest2/mavtest2_0.db3']
    
    # Define waypoints (same for all trajectories)
    waypoints = np.array([
        [5.0, 20.0, 0.0],
        [5.0, 5.0, 0.0],
        [15.0, 5.0, 0.0],
        [15.0, 20.0, 0.0],
        [5.0, 20.0, 0.0]
    ])
    
    # Create figure with subplots based on number of bags
    fig, axes = plt.subplots(1, len(bag_paths), figsize=(7*len(bag_paths), 6), squeeze=False)
    
    # Process each bag and plot in its own subplot
    for i, bag_path in enumerate(bag_paths):
        print(f"Processing bag: {bag_path}")
        # Get trajectory data
        x_vals, y_vals = process_bag(bag_path)
        
        # Get the current axis
        ax = axes[0, i]
        
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
        ax.set_title(f'Trajectory {i+1}')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.grid(True)
        ax.legend()
        ax.axis('equal')
        
        # Invert Y-axis for top-left origin
        ax.invert_yaxis()  # Y increases downward
    
    # Adjust layout and add a main title
    plt.tight_layout()
    fig.suptitle('Ship Trajectories and Waypoints', fontsize=16, y=1.05)
    
    # Save plot
    plt.savefig('trajectories_plot.png', bbox_inches='tight')
    print("Plot saved as trajectories_plot.png")
    # plt.show()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()