#!/usr/bin/env python3

import subprocess
import os
import yaml
import random
import math
import time
import signal
import sys
import shutil
import argparse
import select

# --- Configuration ---
DEFAULT_NUM_RUNS = 50
DEFAULT_NUM_WAYPOINTS = 4  # Including start and end [0,0,0]
DEFAULT_X_RANGE = (-10.0, 10.0)
DEFAULT_Y_RANGE = (-10.0, 10.0)
MIN_WAYPOINT_DISTANCE = 10.0 # Minimum distance between consecutive waypoints
MAX_RETRIES_PER_WAYPOINT = 100 # Max attempts to find a suitable waypoint
DEFAULT_Z_COORD = 0.0
# Use relative paths assuming script is run from workspace root
GUIDANCE_FILE_PATH = "/workspaces/mavlab/inputs/sookshma/guidance.yml"
OUTPUT_BASE_DIR = "/workspaces/mavlab/data/data_from_sim_noNoise"
VESSEL_NAME = "sookshma"
VESSEL_ID = 0
# Topics based on class_guidance_control.py and common usage
TOPICS_TO_RECORD = [
    f"/{VESSEL_NAME}_{VESSEL_ID:02d}/odometry_sim",
    f"/{VESSEL_NAME}_{VESSEL_ID:02d}/actuator_cmd",
    f"/{VESSEL_NAME}_{VESSEL_ID:02d}/waypoints"
]
STARTUP_WAIT_TIME_S = 10  # Increased wait time for ROS nodes to initialize
SHUTDOWN_WAIT_TIME_S = 10 # Time to wait for graceful shutdown
MONITOR_TIMEOUT_S = 300 # Maximum time to wait for simulation completion message

# --- Helper Functions ---

def generate_random_waypoints(num_waypoints, x_range, y_range, z_coord):
    """Generates a list of random waypoints with integer coordinates, starting and ending at [0,0,0], ensuring minimum distance."""
    if num_waypoints < 2:
        raise ValueError("Number of waypoints must be at least 2 (start and end).")

    waypoints = [[0, 0, 0]]  # Start at origin (integers)

    # Generate intermediate waypoints
    for i in range(num_waypoints - 2):
        retries = 0
        while retries < MAX_RETRIES_PER_WAYPOINT:
            # Generate candidate integer coordinates
            x = random.randint(int(x_range[0]), int(x_range[1]))
            y = random.randint(int(y_range[0]), int(y_range[1]))
            candidate_wp = [x, y, int(z_coord)]

            # Get the previous waypoint
            previous_wp = waypoints[-1]

            # Calculate Euclidean distance in XY plane
            dist = math.sqrt((candidate_wp[0] - previous_wp[0])**2 + (candidate_wp[1] - previous_wp[1])**2)

            if dist >= MIN_WAYPOINT_DISTANCE:
                waypoints.append(candidate_wp)
                # print(f"  Accepted waypoint {i+1}: {candidate_wp} (Dist: {dist:.2f})")
                break # Found a suitable waypoint
            # else:
            #     print(f"  Rejected waypoint {i+1} attempt {retries+1}: {candidate_wp} (Dist: {dist:.2f})")

            retries += 1
        else:
            # This else block executes if the while loop completes without a break
            raise RuntimeError(f"Could not find a suitable waypoint after {MAX_RETRIES_PER_WAYPOINT} retries. "
                             f"Check ranges {x_range}, {y_range} and MIN_WAYPOINT_DISTANCE {MIN_WAYPOINT_DISTANCE}.")

    # Ensure the final waypoint [0,0,0] is far enough from the last generated waypoint
    final_wp = [0, 0, 0]
    retries = 0
    while retries < MAX_RETRIES_PER_WAYPOINT:
        last_generated_wp = waypoints[-1]
        dist_to_final = math.sqrt((final_wp[0] - last_generated_wp[0])**2 + (final_wp[1] - last_generated_wp[1])**2)

        if dist_to_final >= MIN_WAYPOINT_DISTANCE:
            waypoints.append(final_wp) # Add the final origin point
            break # Distance is sufficient
        else:
            # Regenerate the *last intermediate* waypoint if the final [0,0,0] is too close
            print(f"Warning: Final waypoint [0,0,0] is too close (Dist: {dist_to_final:.2f}) to {last_generated_wp}. Regenerating last intermediate waypoint...")
            waypoints.pop() # Remove the problematic last intermediate waypoint
            # We need to find a NEW intermediate waypoint that is far from the second-to-last AND allows [0,0,0] to be far from it.

            # Find a suitable replacement for the last *intermediate* waypoint
            replacement_found = False
            for _ in range(MAX_RETRIES_PER_WAYPOINT): # Inner retry loop for replacement
                 x_rep = random.randint(int(x_range[0]), int(x_range[1]))
                 y_rep = random.randint(int(y_range[0]), int(y_range[1]))
                 candidate_replacement_wp = [x_rep, y_rep, int(z_coord)]

                 # Check distance from second-to-last waypoint
                 second_last_wp = waypoints[-1]
                 dist_from_second_last = math.sqrt((candidate_replacement_wp[0] - second_last_wp[0])**2 + (candidate_replacement_wp[1] - second_last_wp[1])**2)

                 # Check distance from replacement to final [0,0,0]
                 dist_from_replacement_to_final = math.sqrt((final_wp[0] - candidate_replacement_wp[0])**2 + (final_wp[1] - candidate_replacement_wp[1])**2)

                 if dist_from_second_last >= MIN_WAYPOINT_DISTANCE and dist_from_replacement_to_final >= MIN_WAYPOINT_DISTANCE:
                     waypoints.append(candidate_replacement_wp)
                     print(f"  Found replacement intermediate waypoint: {candidate_replacement_wp}")
                     replacement_found = True
                     break # Found a suitable replacement

            if not replacement_found:
                 raise RuntimeError(f"Could not find a suitable *replacement* intermediate waypoint after {MAX_RETRIES_PER_WAYPOINT} retries.")

        retries += 1 # Increment outer loop retry counter
    else:
        raise RuntimeError(f"Could not satisfy final waypoint distance constraint after {MAX_RETRIES_PER_WAYPOINT} attempts to regenerate the last intermediate waypoint.")

    return waypoints

def update_guidance_file(waypoints, file_path):
    """Updates the guidance YAML file with new waypoints."""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            if data is None: # Handle empty file case
                data = {}
    except FileNotFoundError:
        print(f"Warning: Guidance file {file_path} not found. Creating a new one.")
        data = {}
    except yaml.YAMLError as e:
        print(f"Error reading YAML file {file_path}: {e}")
        raise

    data['waypoints'] = waypoints
    # Ensure waypoints_type exists, default to XYZ if not present
    if 'waypoints_type' not in data:
        data['waypoints_type'] = 'XYZ'

    try:
        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=None)
        # print(f"Updated guidance file: {file_path}")
    except IOError as e:
        print(f"Error writing to YAML file {file_path}: {e}")
        raise

def terminate_process(process, name, wait_time):
    """Attempts to gracefully terminate a process, escalating if needed."""
    if process and process.poll() is None:
        print(f"Sending SIGINT to {name} (PID: {process.pid})...")
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=wait_time)
            print(f"{name} terminated gracefully.")
        except subprocess.TimeoutExpired:
            print(f"SIGINT timed out for {name}. Sending SIGTERM...")
            process.terminate()
            try:
                process.wait(timeout=wait_time / 2)
                print(f"{name} terminated via SIGTERM.")
            except subprocess.TimeoutExpired:
                print(f"SIGTERM timed out for {name}. Sending SIGKILL...")
                process.kill()
                process.wait()
                print(f"{name} killed.")
        except Exception as e:
             print(f"Error waiting for {name} process: {e}")


# --- Main Execution ---

def main(num_runs, num_waypoints, x_range, y_range, z_coord):
    """Runs the batch simulation and recording process."""

    if not os.path.exists(OUTPUT_BASE_DIR):
        # Create the base output dir if it doesn't exist
        try:
            os.makedirs(OUTPUT_BASE_DIR)
            print(f"Created output directory: {OUTPUT_BASE_DIR}")
        except OSError as e:
            print(f"Error: Could not create base output directory {OUTPUT_BASE_DIR}: {e}")
            sys.exit(1)

    # Check if ROS 2 environment seems sourced (basic check)
    if "ROS_DISTRO" not in os.environ:
        print("Error: ROS 2 environment does not seem to be sourced.")
        print("Please source your ROS 2 setup file (e.g., 'source /opt/ros/<distro>/setup.bash') and try again.")
        sys.exit(1)

    for run_index in range(1, num_runs + 1):
        print(f"\n{'='*10} Starting Run {run_index}/{num_runs} {'='*10}")

        launch_process = None
        record_process = None
        # Define run_output_dir early, even if operations fail later
        run_output_dir = os.path.join(OUTPUT_BASE_DIR, f"run{run_index}")

        try:
            # 1. Prepare Output Directory *before* generating/updating files
            # This makes run_output_dir available even if subsequent steps fail
            print(f"Preparing output directory: {run_output_dir}")
            if os.path.exists(run_output_dir):
                 print(f"Warning: Output directory {run_output_dir} already exists.")
                 # shutil.rmtree(run_output_dir) # Optional: Uncomment to clean before run
            os.makedirs(run_output_dir, exist_ok=True)
            bag_output_path_prefix = os.path.join(run_output_dir, f"sim_data_run{run_index}")

            # 2. Generate Waypoints & Update Config
            print("Generating random waypoints...")
            waypoints = generate_random_waypoints(num_waypoints, x_range, y_range, z_coord)
            print(f"Waypoints: {waypoints}")
            update_guidance_file(waypoints, GUIDANCE_FILE_PATH) # Now if this fails, run_output_dir exists

            # 3. Save Waypoints
            waypoints_save_path = os.path.join(run_output_dir, "waypoints.yaml")
            try:
                with open(waypoints_save_path, 'w') as f:
                    yaml.dump({'waypoints': waypoints, 'waypoints_type': 'XYZ'}, f, default_flow_style=None)
                print(f"Saved waypoints for run {run_index} to {waypoints_save_path}")
            except IOError as e:
                print(f"Error saving waypoints file {waypoints_save_path}: {e}")
                # Decide if we should continue the run or not
                # continue # Example: Skip to next run if saving fails
            except yaml.YAMLError as e:
                 print(f"Error formatting waypoints YAML {waypoints_save_path}: {e}")
                 # continue

            # 4. Launch Simulation
            launch_cmd = ["ros2", "launch", "gnc", "gnc_launch.py"]
            print(f"Executing command: {' '.join(launch_cmd)}")
            # Use Popen to run in background and capture output
            launch_process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, # Capture stderr as well
                text=True,
                bufsize=1,  # Line buffered
                universal_newlines=True # Ensures text mode works consistently
            )
            print(f"Launched simulation (PID: {launch_process.pid}). Waiting {STARTUP_WAIT_TIME_S}s for nodes to initialize...")
            time.sleep(STARTUP_WAIT_TIME_S)

             # Check if launch process died early
            if launch_process.poll() is not None:
                print("Error: Simulation launch process terminated prematurely.")
                stdout, stderr = launch_process.communicate()
                print("--- Launch STDOUT ---")
                print(stdout)
                print("--- Launch STDERR ---")
                print(stderr)
                continue # Skip to next run

            # 5. Start Recording
            record_cmd = ["ros2", "bag", "record", "-a", "-o", bag_output_path_prefix] 
            print(f"Executing command: {' '.join(record_cmd)}")
            record_process = subprocess.Popen(
                record_cmd,
                stdout=subprocess.PIPE, # Capture stdout/stderr if needed
                stderr=subprocess.PIPE,
                text=True
            )
            print(f"Started recording ROS bag (PID: {record_process.pid})...")
            time.sleep(2) # Small delay to ensure recording starts

            if record_process.poll() is not None:
                print("Error: ROS bag recording process terminated prematurely.")
                stdout, stderr = record_process.communicate()
                print("--- Record STDOUT ---")
                print(stdout)
                print("--- Record STDERR ---")
                print(stderr)
                terminate_process(launch_process, "Simulation Launch", SHUTDOWN_WAIT_TIME_S)
                continue # Skip to next run

            # 6. Monitor for Completion
            # The guidance controller logs when it *reaches* a waypoint.
            # The index of the last waypoint is num_waypoints - 1.
            # Use the final completion message logged by class_guidance_control.py
            completion_message = "***All waypoints reached***"
            print(f"Waiting for completion message: '{completion_message}' (Timeout: {MONITOR_TIMEOUT_S}s)")

            start_monitor_time = time.time()
            completion_found = False

            # Use select for non-blocking read on stdout
            stdout_fd = launch_process.stdout.fileno()
            poller = select.poll()
            poller.register(stdout_fd, select.POLLIN)

            while time.time() - start_monitor_time < MONITOR_TIMEOUT_S:
                # Check if launch process ended
                if launch_process.poll() is not None:
                    print("Simulation process ended unexpectedly during monitoring.")
                    break

                # Check if there's output to read (non-blocking)
                if poller.poll(100): # Poll for 100ms
                    try:
                        line = launch_process.stdout.readline()
                        if not line: # End of stream
                            print("Simulation stdout stream ended.")
                            break
                        print(f"[SIM LOG] {line.strip()}") # Print simulation output
                        if completion_message in line:
                            print(f"*** Completion message found! ***")
                            completion_found = True
                            break
                    except Exception as e:
                         print(f"Error reading simulation output: {e}")
                         break
                # Yield CPU briefly
                time.sleep(0.01)


            if not completion_found:
                 if time.time() - start_monitor_time >= MONITOR_TIMEOUT_S:
                     print("Warning: Timeout reached while waiting for completion message.")
                 else:
                     print("Warning: Monitoring stopped before completion message was found (process likely ended).")


            # 7. Stop Recording
            terminate_process(record_process, "ROS Bag Record", SHUTDOWN_WAIT_TIME_S)

            # Find the actual bag directory created by ros2 bag
            # ROS2 bag creates a directory named like bag_output_path_prefix_0
            actual_bag_dir = None
            if os.path.exists(run_output_dir):
                 potential_dirs = [d for d in os.listdir(run_output_dir) if os.path.isdir(os.path.join(run_output_dir, d)) and d.startswith(os.path.basename(bag_output_path_prefix))]
                 if potential_dirs:
                     # Sort to get the _0, _1 etc. in order, pick the first one usually
                     potential_dirs.sort()
                     # The actual bag dir is the one ROS creates, e.g., sim_data_runX_0
                     actual_bag_dir = os.path.join(run_output_dir, potential_dirs[0])
                     print(f"ROS Bag saved in: {actual_bag_dir}")
                 else:
                     print(f"Warning: Could not find created ROS bag directory starting with '{os.path.basename(bag_output_path_prefix)}' in {run_output_dir}")
            else:
                 print(f"Warning: Output directory {run_output_dir} does not exist after recording.")


            print(f"--- Run {run_index} finished ---")

        except Exception as e:
            print(f"\n{'!'*10} An error occurred during run {run_index}: {e} {'!'*10}")
            import traceback
            traceback.print_exc()

        finally:
            # 7. Ensure Processes are Stopped (Cleanup)
            print("--- Cleaning up processes ---")
            terminate_process(record_process, "ROS Bag Record", SHUTDOWN_WAIT_TIME_S / 2) # Shorter wait on cleanup
            terminate_process(launch_process, "Simulation Launch", SHUTDOWN_WAIT_TIME_S) # Give launch more time
            print("--- Cleanup complete ---")
            time.sleep(2) # Small delay before next run

    print(f"{'='*10} Batch simulation completed for {num_runs} runs {'='*10}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run multiple ROS2 simulations with random waypoints and record ROS bags.")
    parser.add_argument("--num_runs", type=int, default=DEFAULT_NUM_RUNS,
                        help=f"Number of simulation runs to perform (default: {DEFAULT_NUM_RUNS})")
    parser.add_argument("--num_waypoints", type=int, default=DEFAULT_NUM_WAYPOINTS,
                        help=f"Number of waypoints per run (including start/end at [0,0,0]) (default: {DEFAULT_NUM_WAYPOINTS})")
    parser.add_argument("--x_min", type=float, default=DEFAULT_X_RANGE[0], help=f"Minimum x coordinate for random waypoints (default: {DEFAULT_X_RANGE[0]})")
    parser.add_argument("--x_max", type=float, default=DEFAULT_X_RANGE[1], help=f"Maximum x coordinate for random waypoints (default: {DEFAULT_X_RANGE[1]})")
    parser.add_argument("--y_min", type=float, default=DEFAULT_Y_RANGE[0], help=f"Minimum y coordinate for random waypoints (default: {DEFAULT_Y_RANGE[0]})")
    parser.add_argument("--y_max", type=float, default=DEFAULT_Y_RANGE[1], help=f"Maximum y coordinate for random waypoints (default: {DEFAULT_Y_RANGE[1]})")
    parser.add_argument("--z_coord", type=float, default=DEFAULT_Z_COORD, help=f"Fixed z coordinate for all waypoints (default: {DEFAULT_Z_COORD})")

    args = parser.parse_args()

    if args.num_waypoints < 2:
        print("Error: --num_waypoints must be at least 2.")
        sys.exit(1)

    x_range = (args.x_min, args.x_max)
    y_range = (args.y_min, args.y_max)

    # Ensure PyYAML is installed
    try:
        import yaml
    except ImportError:
        print("Error: PyYAML is required but not installed.")
        print("Please install it using: pip install PyYAML")
        sys.exit(1)


    main(args.num_runs, args.num_waypoints, x_range, y_range, args.z_coord) 