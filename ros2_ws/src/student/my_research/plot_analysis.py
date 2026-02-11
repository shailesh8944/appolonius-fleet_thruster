# import numpy as np
# import matplotlib.pyplot as plt
# import os

# def plot_ship_analysis(states, commanded_rudders, commanded_props, name, save_prefix, plots_dir):
#     """
#     Enhanced analysis plots for ship motion with proper directory handling
#     """
#     # Create time array
#     time = np.arange(len(commanded_rudders)) * 0.1
#     states = states[:len(commanded_rudders)]
    
#     # Plot 1: Velocity Analysis
#     plt.figure(figsize=(12, 8))
#     plt.subplot(2, 1, 1)
#     plt.plot(time, states[:, 0], 'b-', label='Surge velocity (u)')
#     plt.plot(time, states[:, 1], 'r--', label='Sway velocity (v)')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Velocity (m/s)')
#     plt.title(f'{name} - Velocity Analysis')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(2, 1, 2)
#     plt.plot(time, states[:, 2], 'g-', label='Yaw rate (r)')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Yaw rate (rad/s)')
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.savefig(os.path.join(plots_dir, f'{save_prefix}_velocity_analysis.png'))
#     plt.close()
    
#     # Plot 2: Control Inputs
#     plt.figure(figsize=(12, 8))
#     plt.subplot(2, 1, 1)
#     plt.plot(time, commanded_props, 'b-', label='Commanded Propeller')
#     #plt.plot(time, states[:, 7], 'r--', label='Actual Propeller')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Propeller RPM')
#     plt.title(f'{name} - Propeller Analysis')
#     plt.grid(True)
#     plt.legend()
    
#     plt.subplot(2, 1, 2)
#     plt.plot(time, np.degrees(commanded_rudders), 'b-', label='Commanded Rudder')
#     #plt.plot(time, states[:, 6], 'r--', label='Actual Rudder')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Rudder Angle (deg)')  # Changed to degrees for better readability
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.savefig(os.path.join(plots_dir, f'{save_prefix}_control_analysis.png'))
#     plt.close()
    
#     # Plot 3: Position and Trajectory
#     plt.figure(figsize=(10, 10))
#     plt.plot(states[:, 3], states[:, 4], 'b-', label='Trajectory')
#     plt.plot(states[0, 3], states[0, 4], 'go', label='Start')
#     plt.plot(states[-1, 3], states[-1, 4], 'ro', label='End')
#     plt.xlabel('X Position (m)')
#     plt.ylabel('Y Position (m)')
#     plt.title(f'{name} - Trajectory')
#     plt.grid(True)
#     plt.legend()
#     plt.axis('equal')
#     plt.savefig(os.path.join(plots_dir, f'{save_prefix}_trajectory.png'))
#     plt.close()
    
#     # Print analysis information
#     print(f"\n=== {name} Analysis ===")
#     print(f"Initial position: ({states[0, 3]:.1f}, {states[0, 4]:.1f})")
#     print(f"Final position: ({states[-1, 3]:.1f}, {states[-1, 4]:.1f})")
#     print(f"Average velocity: {np.mean(np.sqrt(states[:, 0]**2 + states[:, 1]**2)):.2f} m/s")
#     print(f"Average propeller: {np.mean(commanded_props):.1f} RPM")
#     print(f"Distance traveled: {np.sum(np.sqrt(np.diff(states[:, 3])**2 + np.diff(states[:, 4])**2)):.2f} m")

# def generate_analysis_plots(states_pursuer1, states_pursuer2, states_pursuer3, 
#                           states_evader, commanded_rudders_p1, commanded_rudders_p2,
#                           commanded_rudders_p3, commanded_rudders_e,
#                           commanded_props_p1, commanded_props_p2,
#                           commanded_props_p3, commanded_props_e, plots_dir="plots"):
#     """Generate and save all analysis plots in the specified directory"""
    
#     # Ensure plots directory exists
#     os.makedirs(plots_dir, exist_ok=True)
#     print(f"\nSaving plots to: {plots_dir}")
    
#     # Combined trajectory plot
#     plt.figure(figsize=(12, 10))
#     plt.plot(states_pursuer1[:, 3], states_pursuer1[:, 4], 'b-', label='Pursuer 1')
#     plt.plot(states_pursuer2[:, 3], states_pursuer2[:, 4], 'g-', label='Pursuer 2')
#     plt.plot(states_pursuer3[:, 3], states_pursuer3[:, 4], 'r-', label='Pursuer 3')
#     plt.plot(states_evader[:, 3], states_evader[:, 4], 'k-', label='Evader')
    
#     # Add start positions
#     plt.plot(states_pursuer1[0, 3], states_pursuer1[0, 4], 'bo', label='P1 Start')
#     plt.plot(states_pursuer2[0, 3], states_pursuer2[0, 4], 'go', label='P2 Start')
#     plt.plot(states_pursuer3[0, 3], states_pursuer3[0, 4], 'ro', label='P3 Start')
#     plt.plot(states_evader[0, 3], states_evader[0, 4], 'ko', label='E Start')
    
#     plt.xlabel('X Position (m)')
#     plt.ylabel('Y Position (m)')
#     plt.title('Combined Trajectories')
#     plt.grid(True)
#     plt.legend()
#     plt.axis('equal')
#     plt.savefig(os.path.join(plots_dir, 'combined_trajectories.png'))
#     plt.close()
    
#     # Individual analysis plots
#     states_list = [states_pursuer1, states_pursuer2, states_pursuer3, states_evader]
#     rudders_list = [commanded_rudders_p1, commanded_rudders_p2, 
#                     commanded_rudders_p3, commanded_rudders_e]
#     props_list = [commanded_props_p1, commanded_props_p2, 
#                   commanded_props_p3, commanded_props_e]
#     names = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3', 'Evader']
#     prefixes = ['pursuer1', 'pursuer2', 'pursuer3', 'evader']
    
#     for states, rudders, props, name, prefix in zip(states_list, rudders_list, 
#                                                    props_list, names, prefixes):
#         states = np.array(states)
#         rudders = np.array(rudders)
#         props = np.array(props)
        
#         print(f"\nGenerating plots for {name}")
#         plot_ship_analysis(states, rudders, props, name, prefix, plots_dir)
import numpy as np
import matplotlib.pyplot as plt
import os

l = 230  # Characteristic length for non-dimensionalization

def plot_ship_analysis(states, commanded_rudders, commanded_props, name, save_prefix, plots_dir):
    """
    Enhanced analysis plots for ship motion with proper directory handling
    """
    # Create time array
    time = np.arange(len(commanded_rudders)) * 0.1
    states = states[:len(commanded_rudders)]
    
    # Plot 1: Velocity Analysis
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(time, states[:, 0], 'b-', label='Surge velocity (u)')
    plt.plot(time, states[:, 1], 'r--', label='Sway velocity (v)')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title(f'{name} - Velocity Analysis')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(time, states[:, 2], 'g-', label='Yaw rate (r)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw rate (rad/s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f'{save_prefix}_velocity_analysis.png'))
    plt.close()
    
    # Plot 2: Control Inputs
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(time, commanded_props, 'b-', label='Commanded Propeller')
    plt.xlabel('Time (s)')
    plt.ylabel('Propeller RPM')
    plt.title(f'{name} - Propeller Analysis')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(time, np.degrees(commanded_rudders), 'b-', label='Commanded Rudder')
    plt.xlabel('Time (s)')
    plt.ylabel('Rudder Angle (deg)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f'{save_prefix}_control_analysis.png'))
    plt.close()
    
    # Plot 3: Position and Trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(states[:, 3], states[:, 4], 'b-', label='Trajectory')
    plt.plot(states[0, 3], states[0, 4], 'go', label='Start')
    plt.plot(states[-1, 3], states[-1, 4], 'ro', label='End')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title(f'{name} - Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(plots_dir, f'{save_prefix}_trajectory.pdf'))
    plt.close()
    
    # Print analysis information
    print(f"\n=== {name} Analysis ===")
    print(f"Initial position: ({states[0, 3]:.1f}, {states[0, 4]:.1f})")
    print(f"Final position: ({states[-1, 3]:.1f}, {states[-1, 4]:.1f})")
    print(f"Average velocity: {np.mean(np.sqrt(states[:, 0]**2 + states[:, 1]**2)):.2f} m/s")
    print(f"Average propeller: {np.mean(commanded_props):.1f} RPM")
    print(f"Distance traveled: {np.sum(np.sqrt(np.diff(states[:, 3])**2 + np.diff(states[:, 4])**2)):.2f} m")
def plot_control_comparison(time, states_pursuers, states_evader,
                          commanded_rudders_pursuers, commanded_rudders_evader,
                          plots_dir="plots"):
    """Plot actual vs commanded rudder angles"""
    plt.figure(figsize=(12, 10))
    colors = ['blue', 'green', 'magenta']
    
    # Create subplots for each pursuer
    for i, (state, cmd_rudder) in enumerate(zip(states_pursuers, commanded_rudders_pursuers)):
        plt.subplot(4, 1, i+1)
        
        # Plot commanded and actual rudder angles
        actual_rudder = np.degrees(state[:len(cmd_rudder), 6])  # Rudder angle is at index 6
        plt.plot(time[:len(cmd_rudder)], np.degrees(cmd_rudder), 
                color=colors[i], linestyle='-', 
                label=f'Commanded Rudder P{i+1}', linewidth=2)
        plt.plot(time[:len(cmd_rudder)], np.degrees(actual_rudder), 
                color=colors[i], linestyle='--',
                label=f'Actual Rudder P{i+1}', linewidth=2)
        
        # Add horizontal lines for rudder limits
        plt.axhline(y=35, color='k', linestyle=':', alpha=0.3)
        plt.axhline(y=-35, color='k', linestyle=':', alpha=0.3)
        
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.ylabel('Rudder Angle (deg)', fontsize=10)
        plt.legend(fontsize=9)
        plt.title(f'Pursuer {i+1} Rudder Response', fontsize=12)
    
    # Add evader subplot
    plt.subplot(4, 1, 4)
    actual_rudder_e = np.degrees(states_evader[:len(commanded_rudders_evader), 6])
    plt.plot(time[:len(commanded_rudders_evader)], np.degrees(commanded_rudders_evader),
            'red', linestyle='-', label='Commanded Rudder Evader', linewidth=2)
    plt.plot(time[:len(commanded_rudders_evader)], np.degrees(actual_rudder_e),
            'red', linestyle='--', label='Actual Rudder Evader', linewidth=2)
    
    plt.axhline(y=35, color='k', linestyle=':', alpha=0.3)
    plt.axhline(y=-35, color='k', linestyle=':', alpha=0.3)
    
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Rudder Angle (deg)', fontsize=10)
    plt.legend(fontsize=9)
    plt.title('Evader Rudder Response', fontsize=12)
    
    plt.tight_layout()
    
    # Save the plot
    os.makedirs(plots_dir, exist_ok=True)
    save_path = os.path.join(plots_dir, 'rudder_response_comparison.pdf')
    plt.savefig(save_path, dpi=400, bbox_inches='tight', 
                facecolor='white', edgecolor='none')
    print(f"Rudder response comparison saved to: {save_path}")
    plt.close()
def plot_group_occupied_angle(time, theta_G_history, theta_vals_history, plots_dir="plots"):
    """
    Plot the evolution of group occupied angle and individual occupied angles
    
    Args:
        time: Array of simulation time steps
        theta_G_history: List of group occupied angles over time
        theta_vals_history: List of lists containing individual occupied angles
    """
    plt.figure(figsize=(12, 8))
    
    # Plot group occupied angle
    plt.subplot(2, 1, 1)
    plt.plot(time, np.degrees(theta_G_history), 'k-', 
             label='Group Occupied Angle (θG)', linewidth=2)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Evolution of Group Occupied Angle')
    plt.legend()
    
    # Plot individual occupied angles
    plt.subplot(2, 1, 2)
    colors = ['blue', 'green', 'magenta']
    labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
    
    for i in range(len(theta_vals_history[0])):
        individual_angles = [angles[i] for angles in theta_vals_history]
        plt.plot(time, np.degrees(individual_angles), 
                color=colors[i], label=labels[i], linewidth=2)
    
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Evolution of Individual Occupied Angles')
    plt.legend()
    
    plt.tight_layout()
    
    # Save the plot
    save_path = os.path.join(plots_dir, 'occupied_angles.pdf')
    plt.savefig(save_path, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"Occupied angles plot saved to: {save_path}")
    plt.close()
def generate_analysis_plots(states_pursuers, states_evader, commanded_rudders_pursuers,
                          commanded_rudders_evader, commanded_props_pursuers,
                          commanded_props_evader, plots_dir="plots"):
    """
    Generate and save all analysis plots in the specified directory
    Now accepts lists of pursuer states and commands for flexible number of pursuers
    All positions and distances are non-dimensionalized by dividing by l
    """
    # Ensure plots directory exists
    os.makedirs(plots_dir, exist_ok=True)
    print(f"\nSaving plots to: {plots_dir}")
    
    # Combined trajectory plot
    plt.figure(figsize=(12, 10))
    
    # Plot pursuer trajectories
    colors = plt.cm.rainbow(np.linspace(0, 1, len(states_pursuers)))
    for i, (states, color) in enumerate(zip(states_pursuers, colors)):
        # Non-dimensionalize positions
        x_pos = states[:, 3] / l
        y_pos = states[:, 4] / l
        plt.plot(x_pos, y_pos, '-', color=color, label=f'Pursuer {i+1}')
        plt.plot(x_pos[0], y_pos[0], 'o', color=color, label=f'P{i+1} Start')
    
    # Plot evader trajectory
    x_pos_e = states_evader[:, 3] / l
    y_pos_e = states_evader[:, 4] / l
    plt.plot(x_pos_e, y_pos_e, 'k-', label='Evader')
    plt.plot(x_pos_e[0], y_pos_e[0], 'ko', label='E Start')
    
    plt.xlabel('X Position (L)')
    plt.ylabel('Y Position (L)')
    plt.title('Combined Trajectories')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(plots_dir, 'combined_trajectories.pdf'))
    plt.close()
    
    # Individual analysis plots for pursuers
    for i, (states, rudders, props) in enumerate(zip(states_pursuers, 
                                                    commanded_rudders_pursuers,
                                                    commanded_props_pursuers)):
        name = f'Pursuer {i+1}'
        prefix = f'pursuer{i+1}'
        print(f"\nGenerating plots for {name}")
        plot_ship_analysis(states, rudders, props, name, prefix, plots_dir)
    
    # Analysis plot for evader
    print("\nGenerating plots for Evader")
    plot_ship_analysis(states_evader, commanded_rudders_evader, 
                      commanded_props_evader, 'Evader', 'evader', plots_dir)
    time_step = 0.5  # Simulation time step
    time = np.arange(len(commanded_rudders_pursuers[0])) * time_step
    plot_control_comparison(time,
                          states_pursuers,
                          states_evader,
                          commanded_rudders_pursuers,
                          commanded_rudders_evader,
                          plots_dir)


# def plot_beta_coefficients(time, beta_values_per_pursuer, plots_dir="plots"):
#     """Plot beta coefficients for all pursuers"""
#     plt.figure(figsize=(10, 6))
#     colors = ['blue', 'green', 'magenta']
#     labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
    
#     for i, betas in enumerate(beta_values_per_pursuer):
#         plt.plot(time, betas, color=colors[i], label=labels[i], linewidth=2)
    
#     plt.grid(True, linestyle='--', alpha=0.7)
#     plt.xlabel('Time (s)', fontsize=12)
#     plt.ylabel('Beta Coefficient (rad)', fontsize=12)
#     plt.title('Evolution of Beta Coefficients', fontsize=14)
#     plt.legend()
#     plt.tight_layout()
    
#     # Save the plot
#     os.makedirs(plots_dir, exist_ok=True)
#     plt.savefig(os.path.join(plots_dir, 'beta_coefficients.pdf'), 
#                 dpi=300, bbox_inches='tight', 
#                 facecolor='white', 
#                 edgecolor='none')
#     plt.close()
def plot_beta_coefficients(time, beta_values_per_pursuer, plots_dir="plots"):
    """Plot beta coefficients for all pursuers"""
    plt.figure(figsize=(10, 6))
    colors = ['blue', 'green', 'magenta']
    labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
    
    # Print debug information
    print(f"Time array shape: {time.shape}")
    for i, betas in enumerate(beta_values_per_pursuer):
        print(f"Beta values shape for Pursuer {i+1}: {betas.shape}")
    
    # Ensure arrays have matching lengths
    min_len = min(len(time), min(len(betas) for betas in beta_values_per_pursuer))
    time_array = time[:min_len]
    
    for i, betas in enumerate(beta_values_per_pursuer):
        plt.plot(time_array, betas[:min_len], color=colors[i], label=labels[i], linewidth=2)
    
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Beta Coefficient', fontsize=12)
    plt.title('Evolution of Beta Coefficients\n(0: Pure Hunting, 1: Pure Surrounding)', fontsize=14)
    plt.legend()
    plt.tight_layout()
    
    # Save the plot
    os.makedirs(plots_dir, exist_ok=True)
    save_path = os.path.join(plots_dir, 'beta_coefficients.pdf')
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
    print(f"Beta coefficients plot saved to: {save_path}")
    plt.close() 
def plot_pursuer_evader_distances(time, states_pursuers, states_evader, capture_radius, plots_dir="plots"):
    """Plot distances between each pursuer and the evader"""
    plt.figure(figsize=(12, 6))
    
    colors = ['blue', 'green', 'magenta']
    labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
    
    # Calculate and plot distance for each pursuer
    for i, pursuer_states in enumerate(states_pursuers):
        distances = []
        for p_state, e_state in zip(pursuer_states, states_evader):
            # Calculate Euclidean distance
            dist = np.sqrt((p_state[3] - e_state[3])**2 + 
                         (p_state[4] - e_state[4])**2)
            distances.append(dist)
        
        plt.plot(time[:len(distances)], distances, 
                color=colors[i], label=labels[i], linewidth=2)
    
    # Add capture radius line
    plt.axhline(y=capture_radius, color='red', linestyle='--', 
                label='Capture Radius', alpha=0.7)
    
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Distance (m)', fontsize=12)
    plt.title('Distance between Pursuers and Evader', fontsize=14)
    plt.legend(fontsize=10)
    
    # Save the plot
    save_path = os.path.join(plots_dir, 'pursuer_evader_distances.pdf')
    plt.savefig(save_path, dpi=300, bbox_inches='tight', 
                facecolor='white', edgecolor='none')
    print(f"Distance plot saved to: {save_path}")
    plt.close()