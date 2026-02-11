# Odometry Plotting Tools

This directory contains tools for plotting odometry data from ROS2 bag files.

## plot_odometry.py

### Description
A script to visualize ship trajectory data from ROS2 bag files with waypoint overlays. It reads from bag files containing the `/sookshma_00/odometry` topic and plots the x/y coordinates.

### Features
- Plots ship trajectory with waypoints
- Handles multiple bag files in a single image
- Configures coordinate system with top-left origin
- Waypoints are displayed as red stars with dashed line connections

### Usage
```bash
python3 plot_odometry.py [bag1.db3] [bag2.db3] [bag3.db3]...
```

### Examples
Plot a single trajectory:
```bash
python3 plot_odometry.py ./team10_run1/team10_run1_0.db3
```

Plot multiple trajectories:
```bash
python3 plot_odometry.py ./team10_run1/team10_run1_0.db3 ./team10_run2/team10_run2_0.db3 ./team10_run3/team10_run3_0.db3
```

### Dependencies
- rclpy
- rosbag2_py
- matplotlib
- numpy

### Output
The script generates a PNG image named `trajectories_plot.png` containing the plotted data.

## plot_all_teams.py

### Description
A batch processing script that automatically locates and processes all team data folders, creating trajectory plots for each team.

### Features
- Automatically finds all team directories (teams 2-17)
- Processes multiple runs for each team
- Handles teams with varying numbers of runs
- Saves plots in the team_plots directory

### Usage
```bash
python3 plot_all_teams.py
```

### How it works
1. Scans the data_from_test directory for team run folders
2. Groups bag files by team number
3. For each team, calls plot_odometry.py with all of that team's bag files
4. Saves the resulting plots in the team_plots directory with filenames like team10_trajectories.png

### Note
The script assumes:
- Team directories follow the naming pattern team{X}_run{Y}
- Each team directory contains a .db3 file
- Teams are numbered from 2 to 17