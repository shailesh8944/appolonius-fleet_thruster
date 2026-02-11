#!/usr/bin/env python3
import os
import subprocess
import re
import glob
from collections import defaultdict

def main():
    # Base directory containing all team data folders
    base_dir = "./"
    output_dir = "./team_plots"
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Regular expression to match team folders
    team_pattern = re.compile(r'team(\d+)_')

    
    # Dictionary to group bag files by team
    team_bags = defaultdict(list)
    # Find all team directories
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        if os.path.isdir(item_path):
            match = team_pattern.match(item)
            if match:
                team_num = int(match.group(1))
                # Only include teams 2-17 as specified
                if 2 <= team_num <= 19:
                    # Find .db3 files in this directory
                    db_files = glob.glob(os.path.join(item_path, "*.db3"))
                    if db_files:
                        # Add the first db3 file (there should typically be only one)
                        team_bags[team_num].append(db_files[0])
    # Process each team's data
    for team_num, bag_files in sorted(team_bags.items()):
        if bag_files:  # Skip teams with no bag files
            print(f"Processing Team {team_num} with {len(bag_files)} runs")
            
            # Sort bag files by run number to ensure consistent ordering
            bag_files.sort(key=lambda x: int(re.search(r'run(\d+)', x).group(1)) 
                           if re.search(r'run(\d+)', x) else 0)
            
            # Build command to run the plot script
            cmd = ["python3", "plot_odometry.py"] + bag_files
            
            # Run the plotting script
            subprocess.run(cmd)
            
            # Move and rename the output file
            if os.path.exists("trajectories_plot.png"):
                os.rename("trajectories_plot.png", 
                         os.path.join(output_dir, f"team{team_num}_trajectories.png"))
                print(f"Saved plot for Team {team_num}")

   

            
            

if __name__ == "__main__":
    main()