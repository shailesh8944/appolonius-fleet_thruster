import numpy as np
def ssa(ang, deg=False):
            """
            Smallest Signed Angle (SSA) function to wrap angle to [-180, 180] degrees or 
            [-pi, pi] radians
            
            Args:
                ang (float): Angle to be wrapped
                deg (bool): Return angle in degrees (default: False)
                
            Returns:
                float: Wrapped angle
            """
            if deg:
                ang = (ang + 180) % 360 - 180
            else:
                ang = (ang + np.pi) % (2 * np.pi) - np.pi
            
            return ang

def compute_local_polar(pursuer_pos, evader_pos):
    dx = evader_pos[0] - pursuer_pos[0]
    dy = evader_pos[1] - pursuer_pos[1]
    r = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx)
    # wrap into [0, 2π)
    alpha = (alpha + 2*np.pi) % (2*np.pi)
    return r, alpha

def sort_pursuers_by_angle(pursuer_states, evader_state):
    
    evader_pos = np.array([evader_state[3], evader_state[4]])
    polar_coords = []
    for i, state in enumerate(pursuer_states):
        px, py = state[3], state[4] 
        r_i, alpha_i = compute_local_polar((px, py), evader_pos)
        polar_coords.append((r_i, alpha_i, i))
    # Sort by alpha
    polar_coords.sort(key=lambda x: x[1])
    sorted_indices = [pc[2] for pc in polar_coords]
    # Return just (r, alpha) sorted
    sorted_polar = [(pc[0], pc[1]) for pc in polar_coords]
    return sorted_indices, sorted_polar

def compute_coverage_angles(sorted_polar, theta_vals):
    
    n = len(sorted_polar)
    alphas = [p[1] for p in sorted_polar]
    epsilons = np.zeros(n)
    for i in range(n):
        i_next = (i + 1) % n
        alpha_i = alphas[i]
        alpha_next = alphas[i_next]
        theta_i = theta_vals[i]
        theta_next = theta_vals[i_next]
        
        diff = alpha_next - alpha_i
        # Adjust if crossing 2π
        if diff > np.pi:
            diff -= 2 * np.pi
        elif diff < -np.pi: 
            diff += 2 * np.pi
        
        epsilons[i] = diff - (theta_next + theta_i) / 2.0
    return epsilons
#def group_agles()
def compute_polygon_maintenance_gradient(pursuer_states, i, R_c, R_f):
    """
    Compute the polygon maintenance gradient (s_i) for pursuer i.
    """
    p_i = np.array(pursuer_states[i][3:5])  # Position of pursuer i
    s_i = np.zeros(2)  # Initialize s_i

    for j, state in enumerate(pursuer_states):
        if i == j:
            continue  # Skip self
        p_j = np.array(state[3:5])  # Position of pursuer j
        d_ij = np.linalg.norm(p_i - p_j)  # Distance between pursuer i and j

        if d_ij <= R_c or d_ij > R_f:
            continue  # Skip if outside the polygon maintenance range

        grad_Q_ij = (
            2 * ((d_ij**2 - R_f**2) / (d_ij**2 - R_c**2 + 1e-8)) *
            ((p_i - p_j) / (d_ij**2 + 1e-8))
        )
        s_i -= grad_Q_ij  # Add the gradient to s_i

    return s_i

def compute_inter_collision_velocity(pursuer_states, i, R_o, R_b):
    """
    Compute the inter-collision avoidance velocity component for pursuer i.
    """
    p_i = np.array(pursuer_states[i][3:5])  # Position of pursuer i
    w_i = np.zeros(2)  # Initialize w_i

    for j, state in enumerate(pursuer_states):
        if i == j:
            continue  # Skip self
        p_j = np.array(state[3:5])  # Position of pursuer j
        d_ij = np.linalg.norm(p_i - p_j)  # Distance between pursuer i and j

        if d_ij <= R_o or d_ij > R_b:
            continue  # Skip if outside the inter-collision range

        grad_U_ij = (
            2 * ((d_ij**2 - R_b**2) / (d_ij**2 - R_o**2 + 1e-8)) *
            ((p_i - p_j) / (d_ij**2 + 1e-8))
        )
        w_i -= grad_U_ij  # Add the gradient to w_i 

    return w_i 
def compute_group_occupied_angle(theta_vals, epsilons):
    """
    Compute the group occupied angle θG as per the formula:
    θG = Σ θi + Σ εi,i+1 where εi,i+1 ≤ 0
    
    Args:
        theta_vals (list): List of individual occupied angles θi
        epsilons (list): List of coverage angles εi,i+1
    
    Returns:
        float: Group occupied angle θG
    """
    n = len(theta_vals)
    theta_sum=0
    # First sum: Σ θi (convert to positive angles)
    for theta in theta_vals:
    
     theta_sum += (abs(theta) )
    
    # Second sum: Add only negative epsilons
    # This represents gaps between pursuers
    eps_sum=0
    for eps in epsilons:
        if eps < 0:  # Changed from <= to < to avoid adding zero values
            eps_sum += eps
    theta_G = theta_sum + eps_sum  # Sum of negative epsilons
    # Debug print
    print(f"Debug - theta_vals: {np.degrees(theta_vals)}")
    print(f"Debug - epsilons: {np.degrees(epsilons)}")
    print(f"Debug - theta_G: {np.degrees(theta_G)}")
    
    return theta_G


class ApolloniusTradeoffController:
    def __init__(self, desired_capture_distance=20.0):
        self.beta_history = {0: [], 1: [], 2: []}  # Track beta values for each pursuer
        self.r_d = desired_capture_distance

    def compute_beta_coefficient(self, i, epsilons, theta_vals, sorted_polar):
        
        n = len(sorted_polar)
        i_next = (i + 1) % n
        i_prev = (i - 1) % n
        
        # Radial distances 
        r_i, _ = sorted_polar[i] 
        r_next, _ = sorted_polar[i_next]
        r_prev, _ = sorted_polar[i_prev] 
        
        # Coverage angles
        eps_i = epsilons[i]
        eps_prev = epsilons[i_prev]
        delta_eps = eps_i - eps_prev  # Δε_i
        
        # Occupied angles for i and i+1
        theta_i = theta_vals[i]
        theta_next = theta_vals[i_next]   
        
        denom = 4.0 * np.pi - theta_next + theta_i
        if denom <= 1e-8:  # Avoid division by zero
            denom = 1e-8
        delta_i = (2.0 * abs(delta_eps)) / denom
        
        sum_r = r_i + r_next + r_prev
        if sum_r <= 1e-8:
            sum_r = 1e-8
        ratio = r_i / sum_r
        exponent = 0.609  # log3(2)=.609
        gamma_i = np.sin(np.pi * (ratio ** exponent))
       
        beta_i = (np.pi / 2.0) * (1.0 - np.exp(-delta_i * gamma_i))
        
        print(f"delta_i: {delta_i:.2f}, gamma_i: {gamma_i:.2f}, beta_i: {beta_i:.2f}")
        return beta_i 


    def compute_tradeoff_velocities(self, sorted_polar, epsilons, V_list, theta_vals, pursuer_states, R_o, R_b, R_c, R_f):
            """
            Compute velocity commands using the new formula for beta_i, inter-collision avoidance, and polygon maintenance.
            """
            n = len(sorted_polar)
            v_commands_sorted = []

            for i in range(n):
                beta_i = self.compute_beta_coefficient(i, epsilons, theta_vals, sorted_polar)
                r_i, alpha_i = sorted_polar[i]
                self.beta_history[i].append(beta_i)
                # Compute hunting and surrounding velocities with increased gains
                h_s = 0.008 # decreased  from 0.008
                v_ih = h_s * r_i * np.array([np.cos(alpha_i), np.sin(alpha_i)])
                alpha_dot = 0.005 * V_list[i] * (epsilons[(i + 1) % n] - epsilons[(i - 1) % n])  # Increased from 0.001
                v_is = alpha_dot * r_i * np.array([-np.sin(alpha_i), np.cos(alpha_i)])

                # Compute inter-collision avoidance gradient (w_i)
                w_i = compute_inter_collision_velocity(pursuer_states, i, R_o, R_b)

                # Compute polygon maintenance gradient (s_i)
                s_i = compute_polygon_maintenance_gradient(pursuer_states, i, R_c, R_f)

                # Compute v_im with increased weight
                b = 0.01  # Increased from 0.1
                v_im = (np.linalg.norm(v_is + v_ih) + b) * np.sign(s_i + w_i)

                # Total velocity with adjusted weights
                v_total =  v_ih +  v_is +   0.5*v_im  # Adjusted weights for better balance

                v_commands_sorted.append(v_total)

                # Debug: Log velocity components
                print(f"Pursuer {i}:")
                print(f"  Alpha: {alpha_i:.2f} rad")
                print(f"  Beta: {beta_i:.2f} rad")
                print(f"  v_ih: {v_ih}, v_is: {v_is}, v_im: {v_im}")
                print(f"  Total velocity: {v_total}")
            

            return v_commands_sorted

    def compute_tradeoff_command(self, pursuer_states, evader_state, V_list, theta_vals, d_c, lambda_min, R_p, R_o, R_b):
        
            n = len(pursuer_states)
            #self.check_convex_polygon_condition(d_c, lambda_min, R_p, n)

            sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, evader_state)
            epsilons = compute_coverage_angles(sorted_polar, theta_vals)
            v_cmds_sorted = self.compute_tradeoff_velocities(sorted_polar, epsilons, V_list, theta_vals, pursuer_states, R_o, R_b,R_c=30, R_f=50)
            #beta_i1 = self.compute_beta_coefficient(i, epsilons, theta_vals, sorted_polar)
            for i in range(len(pursuer_states)):
             beta_i1 = self.compute_beta_coefficient(i, epsilons, theta_vals, sorted_polar)
             self.beta_history[i].append(beta_i1)
            v_cmds = [None] * n
            for idx, v_cmd in zip(sorted_indices, v_cmds_sorted):
                v_cmds[idx] = v_cmd
            
            return v_cmds 
    