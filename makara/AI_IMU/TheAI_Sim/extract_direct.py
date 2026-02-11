#!/usr/bin/env python3
import os
import csv
import numpy as np
import pandas as pd
import argparse
import glob
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
import time

# ROS2 imports
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Import these for standard message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray

# Try to import the custom Actuator message type
try:
    from interfaces.msg import Actuator
    ACTUATOR_MSG_AVAILABLE = True
except ImportError:
    print("Warning: interfaces.msg.Actuator not found. Will use generic deserialization.")
    ACTUATOR_MSG_AVAILABLE = False

def get_message_type(topic_type):
    """
    Get message type from topic type string
    """
    if topic_type == 'nav_msgs/msg/Odometry':
        return Odometry
    elif topic_type == 'sensor_msgs/msg/Imu':
        return Imu
    elif topic_type == 'std_msgs/msg/Float64':
        return Float64
    elif topic_type == 'std_msgs/msg/Float64MultiArray':
        return Float64MultiArray
    else:
        # Use generic message getter if specific import not available
        try:
            return get_message(topic_type)
        except (AttributeError, ModuleNotFoundError) as e:
            print(f"Error getting message type for {topic_type}: {e}")
            return None

def process_bag_directory(bag_dir, output_dir):
    """
    Process a ROS2 bag directory and extract data to CSV files at maximum speed
    
    Args:
        bag_dir: Path to the ROS2 bag directory
        output_dir: Path to save extracted CSV files
    
    Returns:
        Dictionary with extracted dataframes
    """
    start_time = time.time()
    bag_name = os.path.basename(bag_dir)
    
    # Create output directory
    bag_output_dir = os.path.join(output_dir, bag_name)
    os.makedirs(bag_output_dir, exist_ok=True)
    
    # Find the .db3 file
    db3_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db3_files:
        print(f"No .db3 files found in {bag_dir}")
        subdirs = [d for d in os.listdir(bag_dir) if os.path.isdir(os.path.join(bag_dir, d))]
        for subdir in subdirs:
            subdir_path = os.path.join(bag_dir, subdir)
            db3_files = glob.glob(os.path.join(subdir_path, "*.db3"))
            if db3_files:
                bag_dir = subdir_path
                break
    
    if not db3_files:
        raise ValueError(f"No .db3 files found in {bag_dir} or its subdirectories")
    
    print(f"Processing bag directory: {bag_dir}")
    
    # Set up bag reader
    storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    # Initialize ROS2
    if not rclpy.ok():
        rclpy.init()
    
    # Initialize reader
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topic_types}
    
    print(f"Available topics: {list(topic_type_map.keys())}")
    
    # Data dictionaries for each topic type
    imu_data = {}
    odom_data = {}
    rudder_data = {}
    vessel_state_data = {}
    
    # Track topics for data collection
    imu_topic = None
    odom_topic = None
    rudder_topic = None
    actuator_topic = None
    vessel_state_topic = None
    vessel_states_ekf_topic = None
    
    # Find relevant topics
    for topic, msg_type in topic_type_map.items():
        if 'imu/data' in topic:
            imu_topic = topic
        elif 'odometry_sim' in topic:
            odom_topic = topic
        elif 'Rudder' in topic and 'encoder' in topic:
            rudder_topic = topic
        elif 'actuator_cmd' in topic:
            actuator_topic = topic
        elif 'vessel_state' in topic and not 'ekf' in topic:
            vessel_state_topic = topic
        elif 'vessel_states_ekf' in topic:
            vessel_states_ekf_topic = topic
    
    # If no specific rudder topic found, use actuator topic for rudder data
    if rudder_topic is None and actuator_topic is not None:
        rudder_topic = actuator_topic
        print(f"No dedicated rudder topic found, using actuator topic for rudder data: {rudder_topic}")
    
    # Prefer vessel_state over vessel_states_ekf if both are available
    if vessel_state_topic is None and vessel_states_ekf_topic is not None:
        vessel_state_topic = vessel_states_ekf_topic
    
    print(f"Using topics:")
    print(f"  IMU: {imu_topic}")
    print(f"  Odometry: {odom_topic}")
    print(f"  Rudder: {rudder_topic}")
    print(f"  Vessel State: {vessel_state_topic}")
    
    # Read message count for progress reporting
    total_messages = 0
    processed_messages = 0
    
    # Read all messages from the bag (first pass to count messages)
    while reader.has_next():
        reader.read_next()
        total_messages += 1
    
    # Reset reader
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    print(f"Processing {total_messages} messages...")
    progress_interval = max(1, total_messages // 20)  # Show progress every 5%
    
    # Read all messages from the bag
    while reader.has_next():
        topic_name, data, t_stamp = reader.read_next()
        processed_messages += 1
        
        if processed_messages % progress_interval == 0:
            progress_pct = (processed_messages / total_messages) * 100
            elapsed_time = time.time() - start_time
            print(f"Progress: {progress_pct:.1f}% ({processed_messages}/{total_messages}) - Elapsed: {elapsed_time:.2f}s")
        
        # Only process relevant topics for maximum speed
        if topic_name not in [imu_topic, odom_topic, rudder_topic, vessel_state_topic]:
            continue
        
        # Convert timestamp to seconds
        if hasattr(t_stamp, 'sec') and hasattr(t_stamp, 'nanosec'):
            timestamp = t_stamp.sec + (t_stamp.nanosec / 1e9)
        elif isinstance(t_stamp, int):
            # If t_stamp is already an integer (nanoseconds), convert to seconds
            timestamp = t_stamp / 1e9
        else:
            # If we can't determine the timestamp format, skip
            continue
        
        # Process vessel state data for yaw
        if topic_name == vessel_state_topic:
            try:
                msg_type = get_message_type(topic_type_map[topic_name])
                if msg_type:
                    msg = deserialize_message(data, msg_type)
                    
                    # Check if it's a Float64MultiArray message
                    if hasattr(msg, 'data') and len(msg.data) > 11:
                        # Extract yaw (index 11 in the vessel state array)
                        yaw = msg.data[11]
                        vessel_state_data[timestamp] = {'yaw': yaw}
            except Exception as e:
                pass  # Skip errors for maximum speed
        
        # Process IMU data
        elif topic_name == imu_topic:
            try:
                msg_type = get_message_type(topic_type_map[topic_name])
                if msg_type:
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract timestamp
                    sec = msg.header.stamp.sec
                    nanosec = msg.header.stamp.nanosec
                    msg_timestamp = sec + nanosec / 1e9
                    
                    imu_data[msg_timestamp] = {
                        'accel_x': msg.linear_acceleration.x,
                        'accel_y': msg.linear_acceleration.y,
                        'accel_z': msg.linear_acceleration.z,
                        'ang_vel_x': msg.angular_velocity.x,
                        'ang_vel_y': msg.angular_velocity.y,
                        'ang_vel_z': msg.angular_velocity.z
                    }
            except Exception as e:
                pass  # Skip errors for maximum speed
        
        # Process odometry data
        elif topic_name == odom_topic:
            try:
                msg_type = get_message_type(topic_type_map[topic_name])
                if msg_type:
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract timestamp
                    sec = msg.header.stamp.sec
                    nanosec = msg.header.stamp.nanosec
                    msg_timestamp = sec + nanosec / 1e9
                    
                    # Extract position, orientation, velocities
                    pos_x = msg.pose.pose.position.x
                    pos_y = msg.pose.pose.position.y
                    pos_z = msg.pose.pose.position.z
                    
                    vel_x = msg.twist.twist.linear.x
                    vel_y = msg.twist.twist.linear.y
                    vel_z = msg.twist.twist.linear.z
                    
                    odom_data[msg_timestamp] = {
                        'pos_x': pos_x,
                        'pos_y': pos_y,
                        'pos_z': pos_z,
                        'vel_x': vel_x,
                        'vel_y': vel_y,
                        'vel_z': vel_z
                    }
            except Exception as e:
                pass  # Skip errors for maximum speed
        
        # Process Rudder encoder data
        elif topic_name == rudder_topic:
            try:
                msg_type = get_message_type(topic_type_map[topic_name])
                if msg_type:
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract timestamp
                    sec = msg.header.stamp.sec
                    nanosec = msg.header.stamp.nanosec
                    msg_timestamp = sec + nanosec / 1e9
                    
                    # Check if this is an Actuator message (which has no 'data' attribute)
                    if hasattr(msg, 'actuator_values') and hasattr(msg, 'actuator_names'):
                        # Find the cs_1 value
                        rudder_angle = None
                        for i, name in enumerate(msg.actuator_names):
                            if name == 'cs_1' and i < len(msg.actuator_values):
                                rudder_angle = msg.actuator_values[i]
                                break
                        
                        if rudder_angle is not None:
                            rudder_data[msg_timestamp] = {
                                'rudder_angle': rudder_angle
                            }
                    # Standard Float64 message with 'data' attribute
                    elif hasattr(msg, 'data'):
                        rudder_data[msg_timestamp] = {
                            'rudder_angle': msg.data
                        }
            except Exception as e:
                pass  # Skip errors for maximum speed
    
    elapsed_time = time.time() - start_time
    print(f"Processed {processed_messages} messages in {elapsed_time:.2f} seconds ({processed_messages/elapsed_time:.2f} msgs/sec)")
    
    # Convert dictionaries to DataFrames
    dataframes = {}
    
    # Convert data collections to dataframes in parallel
    print("Converting data to DataFrames...")
    
    if imu_data:
        imu_df = pd.DataFrame.from_dict(imu_data, orient='index')
        imu_df.index.name = 'timestamp'
        imu_df.reset_index(inplace=True)
        dataframes['imu'] = imu_df
        imu_df.to_csv(os.path.join(bag_output_dir, 'imu.csv'), index=False)
        print(f"Saved IMU data with {len(imu_df)} samples")
    
    if odom_data:
        odom_df = pd.DataFrame.from_dict(odom_data, orient='index')
        odom_df.index.name = 'timestamp'
        odom_df.reset_index(inplace=True)
        dataframes['odom'] = odom_df
        odom_df.to_csv(os.path.join(bag_output_dir, 'odom.csv'), index=False)
        print(f"Saved odometry data with {len(odom_df)} samples")
    
    if rudder_data:
        rudder_df = pd.DataFrame.from_dict(rudder_data, orient='index')
        rudder_df.index.name = 'timestamp'
        rudder_df.reset_index(inplace=True)
        dataframes['rudder'] = rudder_df
        rudder_df.to_csv(os.path.join(bag_output_dir, 'rudder.csv'), index=False)
        print(f"Saved rudder data with {len(rudder_df)} samples")
    
    if vessel_state_data:
        vessel_state_df = pd.DataFrame.from_dict(vessel_state_data, orient='index')
        vessel_state_df.index.name = 'timestamp'
        vessel_state_df.reset_index(inplace=True)
        dataframes['vessel_state'] = vessel_state_df
        vessel_state_df.to_csv(os.path.join(bag_output_dir, 'vessel_state.csv'), index=False)
        print(f"Saved vessel state data with {len(vessel_state_df)} samples")
    
    # Create synchronized dataset
    if all(topic in dataframes for topic in ['imu', 'odom', 'rudder']) and 'vessel_state' in dataframes:
        print("Synchronizing datasets...")
        # Use odometry timestamps as reference since it has the lowest rate
        reference_timestamps = dataframes['odom']['timestamp'].values
        
        synchronized_data = []
        
        for ref_time in reference_timestamps:
            # Find closest IMU data
            imu_idx = np.argmin(np.abs(dataframes['imu']['timestamp'].values - ref_time))
            imu_time = dataframes['imu'].iloc[imu_idx]['timestamp']
            
            # Find closest rudder data
            rudder_idx = np.argmin(np.abs(dataframes['rudder']['timestamp'].values - ref_time))
            rudder_time = dataframes['rudder'].iloc[rudder_idx]['timestamp']
            
            # Find closest vessel state data for yaw
            vessel_idx = np.argmin(np.abs(dataframes['vessel_state']['timestamp'].values - ref_time))
            vessel_time = dataframes['vessel_state'].iloc[vessel_idx]['timestamp']
            
            # Check if timestamps are within acceptable range (100ms)
            imu_time_diff = abs(imu_time - ref_time)
            rudder_time_diff = abs(rudder_time - ref_time)
            vessel_time_diff = abs(vessel_time - ref_time)
            
            # Only include data points where all sources are close to the reference timestamp
            if max(imu_time_diff, rudder_time_diff, vessel_time_diff) <= 0.1:
                # Get synchronized data
                odom_row = dataframes['odom'][dataframes['odom']['timestamp'] == ref_time].iloc[0]
                imu_row = dataframes['imu'].iloc[imu_idx]
                rudder_row = dataframes['rudder'].iloc[rudder_idx]
                vessel_row = dataframes['vessel_state'].iloc[vessel_idx]
                
                # Combine data
                synchronized_row = {
                    'timestamp': ref_time,
                    'pos_x': odom_row['pos_x'],
                    'pos_y': odom_row['pos_y'],
                    'vel_x': odom_row['vel_x'],
                    'vel_y': odom_row['vel_y'],
                    'yaw': vessel_row['yaw'],
                    'accel_x': imu_row['accel_x'],
                    'accel_y': imu_row['accel_y'],
                    'ang_vel_z': imu_row['ang_vel_z'],
                    'rudder_angle': rudder_row['rudder_angle']
                }
                
                synchronized_data.append(synchronized_row)
        
        if synchronized_data:
            synchronized_df = pd.DataFrame(synchronized_data)
            dataframes['synchronized'] = synchronized_df
            synchronized_df.to_csv(os.path.join(bag_output_dir, 'synchronized.csv'), index=False)
            print(f"Saved synchronized data with {len(synchronized_df)} samples")
            
            # Generate visualization plots if needed (can be disabled for maximum speed)
            visualize_data(dataframes, os.path.join(bag_output_dir, 'plots'))
    
    total_elapsed_time = time.time() - start_time
    print(f"Total processing time: {total_elapsed_time:.2f} seconds")
    return dataframes

def visualize_data(dataframes, output_dir='./plots'):
    """
    Visualize the extracted data
    
    Args:
        dataframes: Dictionary of DataFrames
        output_dir: Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    if 'synchronized' in dataframes:
        print("Generating visualization plots...")
        df = dataframes['synchronized']
        
        # Plot position trajectory
        plt.figure(figsize=(10, 8))
        plt.plot(df['pos_x'], df['pos_y'])
        plt.scatter(df['pos_x'].iloc[0], df['pos_y'].iloc[0], c='green', label='Start')
        plt.scatter(df['pos_x'].iloc[-1], df['pos_y'].iloc[-1], c='red', label='End')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('2D Trajectory')
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, 'trajectory.png'))
        
        # Plot velocities
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(df['timestamp'], df['vel_x'])
        plt.ylabel('X Velocity (m/s)')
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(df['timestamp'], df['vel_y'])
        plt.xlabel('Time (s)')
        plt.ylabel('Y Velocity (m/s)')
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, 'velocities.png'))
        
        # Plot accelerations
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(df['timestamp'], df['accel_x'])
        plt.ylabel('X Acceleration (m/s²)')
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(df['timestamp'], df['accel_y'])
        plt.xlabel('Time (s)')
        plt.ylabel('Y Acceleration (m/s²)')
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, 'accelerations.png'))
        
        # Plot yaw angle
        plt.figure(figsize=(12, 4))
        plt.plot(df['timestamp'], df['yaw'])
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw Angle (rad)')
        plt.title('Vessel Heading')
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, 'yaw.png'))
        
        # Plot rudder angle
        plt.figure(figsize=(12, 4))
        plt.plot(df['timestamp'], df['rudder_angle'])
        plt.xlabel('Time (s)')
        plt.ylabel('Rudder Angle (deg)')
        plt.title('Rudder Control Input')
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, 'rudder.png'))

def main():
    parser = argparse.ArgumentParser(description='Extract data from ROS2 bags using rclpy at maximum speed')
    parser.add_argument('--data_dir', type=str, default='../data_from_sim',
                        help='Directory containing ROS2 bags')
    parser.add_argument('--runs', type=str, default='run*',
                        help='Pattern to select specific run folders (e.g., "run1,run2" or "run*")')
    parser.add_argument('--output_dir', type=str, default='./extracted_data',
                        help='Directory to save extracted data and plots')
    parser.add_argument('--no_plots', action='store_true',
                        help='Skip generating visualization plots')
    parser.add_argument('--max_threads', type=int, default=4,
                        help='Maximum number of parallel threads for processing')
    
    args = parser.parse_args()
    
    # Record the start time
    start_time = time.time()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Find bag directories
    if ',' in args.runs:
        # Specific runs provided as comma-separated list
        run_patterns = args.runs.split(',')
        bag_dirs = []
        for pattern in run_patterns:
            matching_dirs = glob.glob(os.path.join(args.data_dir, pattern))
            bag_dirs.extend(matching_dirs)
    else:
        # Wildcard pattern
        bag_dirs = glob.glob(os.path.join(args.data_dir, args.runs))
    
    if not bag_dirs:
        print(f"No bag directories found matching pattern: {args.runs}")
        return
    
    print(f"Found {len(bag_dirs)} run directories")
    
    # Initialize ROS if not already initialized
    if not rclpy.ok():
        rclpy.init()
    
    # Process each bag and save CSV files
    successful_bag_dirs = []
    successful_csv_files = []
    
    for i, bag_dir in enumerate(bag_dirs):
        try:
            print(f"\nProcessing bag directory {i+1}/{len(bag_dirs)}: {bag_dir}")
            dataframes = process_bag_directory(bag_dir, args.output_dir)
            
            if 'synchronized' in dataframes:
                csv_path = os.path.join(args.output_dir, os.path.basename(bag_dir), "synchronized.csv")
                successful_bag_dirs.append(bag_dir)
                successful_csv_files.append(csv_path)
                print(f"Successfully processed {bag_dir}")
            else:
                print(f"Warning: No synchronized data created for {bag_dir}")
        except Exception as e:
            print(f"Error processing {bag_dir}: {str(e)}")
    
    # Shutdown ROS
    try:
        rclpy.shutdown()
    except:
        pass
    
    # Create a metadata file listing all processed bags and their CSV file paths
    if successful_csv_files:
        metadata_path = os.path.join(args.output_dir, "metadata.csv")
        metadata_df = pd.DataFrame({
            'bag_dir': successful_bag_dirs,
            'csv_path': successful_csv_files
        })
        metadata_df.to_csv(metadata_path, index=False)
        
        elapsed_time = time.time() - start_time
        print(f"\nExtracted data from {len(successful_csv_files)} bag directories in {elapsed_time:.2f} seconds")
        print(f"Average processing time per bag: {elapsed_time / len(bag_dirs):.2f} seconds")
        print(f"Metadata saved to {metadata_path}")
        print(f"All data saved to {args.output_dir}")
    else:
        print("\nNo data was successfully extracted from any bag directory")

if __name__ == '__main__':
    main() 