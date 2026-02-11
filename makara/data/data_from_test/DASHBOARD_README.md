# Waypoint Tracking Analysis Dashboard

This dashboard provides comprehensive analysis and visualization of the waypoint tracking exercise results. It allows students to see how they performed relative to their peers and understand the impact of different gain parameters on tracking performance.

## Features

1. **Dashboard Overview**: A high-level summary of results, including top-performing teams, waypoint tracking statistics, and key correlations between gain parameters and performance.

2. **Leaderboard**: Ranked list of teams based on waypoints tracked and scores, with special emphasis on teams that tracked all 4 waypoints.

3. **Gain Analysis**: Advanced visualization of relationships between different gain parameters (Kp_o, Ki_o, Kp_i, Kd_i) and scores, with recommended parameter ranges for successful tracking. Features include:
   - Interactive scatter plots showing relationships between individual gains and scores
   - Box plots of successful parameter ranges
   - Parameter correlation matrix to identify patterns across multiple gains
   - Distribution analysis of parameter values

4. **Team Comparison**: Interactive tool to compare performance across multiple teams, featuring:
   - Side-by-side comparison of scores and waypoints tracked
   - Radar charts for comparing gain parameter configurations
   - Detailed view of all runs for each team

5. **Trajectory Visualization**: Visual representation of ship trajectories with overlaid waypoints (when available).

6. **Raw Data**: Access to the complete dataset with filtering options and download capability.

## Understanding the Control System

The waypoint tracking exercise used a cascaded PID control system with:

### Outer Loop Controller
Controls the heading of the ship to point toward the target waypoint:
- **Kp_o** (Proportional Gain): Controls how aggressively the ship turns toward the waypoint. Higher values cause more aggressive turning.
- **Ki_o** (Integral Gain): Helps eliminate steady-state error by accumulating error over time. Useful for counteracting consistent disturbances.

### Inner Loop Controller
Controls the rudder to achieve the desired heading:
- **Kp_i** (Proportional Gain): Determines how aggressively the rudder responds to heading errors.
- **Kd_i** (Derivative Gain): Provides damping to prevent overshoot and oscillations. Helps stabilize the ship's motion.

## Installation

1. Ensure you have Python installed (3.8+ recommended)

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Running the Dashboard

From the data_from_test directory, run:

```bash
streamlit run app.py
```

Or use the provided script:

```bash
./run_dashboard.sh
```

The dashboard will be accessible in your web browser at http://localhost:8501.

## Interpreting the Results

### Score Interpretation
- Lower scores indicate better performance (based on tracking accuracy)
- Teams are ranked based on:
  1. Number of waypoints tracked (higher is better)
  2. Score (lower is better)

### Optimal Gain Ranges
The dashboard identifies successful gain ranges based on teams that tracked all 4 waypoints. These ranges can provide guidance for future tuning exercises.

### Common Patterns
Through data analysis, several patterns typically emerge:
- Too high proportional gains often lead to oscillations and instability
- Too low gains may result in sluggish response and inability to track waypoints
- Balanced gain combinations across all four parameters typically yield the best results

## Data Source

The analysis is based on the data in `team_plots/scores.csv`, which contains the results of each team's attempts at tuning their controllers. The trajectory plots are sourced from the `team_plots` directory. 