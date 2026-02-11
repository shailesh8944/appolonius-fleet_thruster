import numpy as np
import matplotlib.pyplot as plt
l=230


def calculate_distances_and_vertices(states_pursuers, R_o, R_b, R_c, R_f):
    """
    Calculate distances between pursuers and polygon vertices over time.
    All distances are non-dimensionalized by dividing by l.
    """
    inter_collision_violations = []
    distance_maintenance_violations = []
    vertex_distances = []  # Store distances between adjacent vertices
    avg_vertex_distances = []  # Store average vertex distance per timestep
    
    n_pursuers = len(states_pursuers)
    
    # Non-dimensionalize the radius parameters
    R_o = R_o / l
    R_b = R_b / l
    R_c = R_c / l
    R_f = R_f / l
    
    for t in range(len(states_pursuers[0])):
        pursuer_positions = [states[t][3:5] for states in states_pursuers]
        n_violations_inter_collision = 0
        n_violations_distance_maintenance = 0
        vertex_dist_t = []
        
        # Calculate distances between all pairs
        for i in range(n_pursuers):
            for j in range(i + 1, n_pursuers):
                d_ij = np.linalg.norm(pursuer_positions[i] - pursuer_positions[j]) / l
                
                # Check violations
                if R_o < d_ij <= R_b:
                    n_violations_inter_collision += 1
                if R_c < d_ij <= R_f:
                    n_violations_distance_maintenance += 1
                    
                # Calculate vertex distances (between adjacent pursuers)
                if j == (i + 1) % n_pursuers:
                    vertex_dist_t.append(d_ij)
        
        inter_collision_violations.append(n_violations_inter_collision)
        distance_maintenance_violations.append(n_violations_distance_maintenance)
        vertex_distances.append(vertex_dist_t)
        avg_vertex_distances.append(np.mean(vertex_dist_t))
    
    return (inter_collision_violations, distance_maintenance_violations, 
            np.array(vertex_distances), np.array(avg_vertex_distances))

def plot_all_distances(time, states_pursuers, R_o, R_b, R_c, R_f, plots_dir):
    """
    Create comprehensive distance analysis plots with non-dimensionalized distances.
    """
    results = calculate_distances_and_vertices(states_pursuers, R_o, R_b, R_c, R_f)
    (inter_violations, maintenance_violations, vertex_distances, avg_vertex_distances) = results
    data_length = len(inter_violations)
    plot_time = time[:data_length]
    
    # Non-dimensionalize radius parameters for plotting
    R_o = R_o / l
    R_b = R_b / l
    R_c = R_c / l
    R_f = R_f / l
    
    # Plot 1: Violations over time
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(plot_time, inter_violations, 'r-', label='Inter-Collision Violations')
    plt.plot(plot_time, maintenance_violations, 'b-', label='Distance Maintenance Violations')
    plt.title('Safety and Formation Violations Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Number of Violations')
    plt.grid(True)
    plt.legend()
    
    # Plot 2: Average vertex distances
    plt.subplot(2, 1, 2)
    plt.plot(plot_time, avg_vertex_distances, 'g-', label='Average Vertex Distance')
    plt.axhline(y=R_o, color='r', linestyle='--', label=f'R_o/L = {R_o:.3f}')
    plt.axhline(y=R_b, color='b', linestyle='--', label=f'R_b/L = {R_b:.3f}')
    plt.axhline(y=R_c, color='y', linestyle='--', label=f'R_c/L = {R_c:.3f}')
    plt.axhline(y=R_f, color='m', linestyle='--', label=f'R_f/L = {R_f:.3f}')
    plt.title('Average Distance Between Adjacent Pursuers')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (L)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(f"{plots_dir}/distance_analysis.png")
    
    # Plot 3: Individual vertex distances
    plt.figure(figsize=(12, 6))
    n_pursuers = len(states_pursuers)
    
    # Ensure we have the correct number of vertex distances
    for i in range(n_pursuers+1):
        if i < vertex_distances.shape[1]:  # Check if index is valid
            plt.plot(plot_time, vertex_distances[:, i], 
                    label=f'Vertex {i}-{(i+1)%n_pursuers}')
    
    plt.axhline(y=R_o, color='r', linestyle='--', label=f'R_o/L ')
    plt.axhline(y=R_b, color='b', linestyle='--', label=f'R_b/L ')
    plt.axhline(y=R_c, color='y', linestyle='--', label=f'R_c/L ')
    plt.axhline(y=R_f, color='m', linestyle='--', label=f'R_f/L ')
    
    plt.title('Distance Between Adjacent Pursuers (Polygon Vertices)')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (L)')
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.savefig(f"{plots_dir}/vertex_distances.pdf")
    
    # Plot 4: Distance distribution histogram
    plt.figure(figsize=(10, 6))
    plt.hist(vertex_distances.flatten(), bins=30, alpha=0.7)
    plt.axvline(x=R_o, color='r', linestyle='--', label=f'R_o/L ')
    plt.axvline(x=R_b, color='b', linestyle='--', label=f'R_b/L ')
    plt.axvline(x=R_c, color='y', linestyle='--', label=f'R_c/L ')
    plt.axvline(x=R_f, color='m', linestyle='--', label=f'R_f/L ')
    plt.title('Distribution of Inter-Pursuer Distances')
    plt.xlabel('Distance (L)')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{plots_dir}/distance_distribution.png")
    
    plt.close('all')

#Add to main simulation code:
from cooperative_pursuer import compute_inter_collision_velocity, compute_polygon_maintenance_gradient    

def plot_gradients_over_time(time, states_pursuers, R_o, R_b, R_c, R_f, plots_dir):
    """
    Plot gradients over time with non-dimensionalized distances.
    """
    n_pursuers = len(states_pursuers)
    n_timesteps = len(time)
    
    # Non-dimensionalize radius parameters
    R_o = R_o 
    R_b = R_b 
    R_c = R_c 
    R_f = R_f 
    
    # Initialize arrays to store gradient magnitudes
    collision_grads = np.zeros((n_timesteps, n_pursuers)) 
    polygon_grads = np.zeros((n_timesteps, n_pursuers))
    
    # Calculate gradients for each timestep and pursuer
    for t in range(n_timesteps):
        pursuer_states_t = [states[t] for states in states_pursuers] 
        
        for i in range(n_pursuers):
            w_i = compute_inter_collision_velocity(pursuer_states_t, i, R_o, R_b)
            s_i = compute_polygon_maintenance_gradient(pursuer_states_t, i, R_c, R_f)
            
            collision_grads[t,i] = np.linalg.norm(w_i)
            polygon_grads[t,i] = np.linalg.norm(s_i)
    
    # Create single figure for merged gradients
    plt.figure(figsize=(12, 8))
    
    # Plot collision gradients with solid lines
    for i in range(n_pursuers):
        plt.plot(time, collision_grads[:,i],
                linestyle='-', 
                label=f'Collision Gradient P{i+1}')
    
    # Plot maintenance gradients with dashed lines
    for i in range(n_pursuers):
        plt.plot(time, polygon_grads[:,i],
                linestyle='--', 
                label=f'Maintenance Gradient P{i+1}')
    
    # Add reference distances as horizontal lines
    plt.axvline(x=R_o, color='r', linestyle=':', label=f'R_o ')
    plt.axvline(x=R_b, color='b', linestyle=':', label=f'R_b')
    plt.axvline(x=R_c, color='y', linestyle=':', label=f'R_c ')
    plt.axvline(x=R_f, color='m', linestyle=':', label=f'R_f ')
    
    plt.title('Combined Inter-Collision and Maintenance Gradients')
    plt.xlabel('Distance (L)')
    plt.ylabel('Time (s)')
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.xlim(0, 1.0)  # Adjusted for non-dimensionalized distances
    
    plt.tight_layout()
    plt.savefig(f"{plots_dir}/merged_gradients.png", bbox_inches='tight', dpi=300)
    plt.close()

    # Print combined statistics
    print("\nCombined Gradient Statistics:") 
    for i in range(n_pursuers):
        print(f"\nPursuer {i+1}:")
        print(f"Collision Gradient    - Mean: {np.mean(collision_grads[:,i]):.2f}, Max: {np.max(collision_grads[:,i]):.2f}")
        print(f"Maintenance Gradient - Mean: {np.mean(polygon_grads[:,i]):.2f}, Max: {np.max(polygon_grads[:,i]):.2f}")

