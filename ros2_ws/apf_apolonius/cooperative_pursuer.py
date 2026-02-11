import numpy as np
DEBUG_LOG = False

# =========================
# Smallest Signed Angle
# =========================
def ssa(ang, deg=False):
    """
    Wrap angle to [-pi, pi] (or [-180, 180] if deg=True).
    """
    if deg:
        ang = (ang + 180.0) % 360.0 - 180.0
    else:
        ang = (ang + np.pi) % (2.0 * np.pi) - np.pi
    return ang


def compute_local_polar(pursuer_pos, evader_pos):
    """
    Polar coordinates of pursuer w.r.t evader.
    pursuer_pos, evader_pos: (x, y)
    Returns:
        r: distance
        alpha: angle in [0, 2pi)
    """
    dx =  evader_pos[0]-pursuer_pos[0] 
    dy = evader_pos[1]-pursuer_pos[1]         
    r = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx)
    alpha = (alpha + 2.0 * np.pi) % (2.0 * np.pi) #  wrapping angle from [-pi,pi] to  [0,2pi]
    return r, alpha


def sort_pursuers_by_angle(pursuer_states, evader_state):
    """
    Sort pursuers around the evader by polar angle (CCW).==========
    
        sorted_indices: indices of pursuers in CCW order
        sorted_polar: list of (r_i, alpha_i) in that order
    """
    e_pos = np.array([evader_state[6], evader_state[7]])
    polar_coords = []
    for i, state in enumerate(pursuer_states):
        px, py = state[6], state[7]
        r_i, alpha_i = compute_local_polar((px, py), e_pos)
        polar_coords.append((r_i, alpha_i, i))     

    polar_coords.sort(key=lambda x: x[1])   # sort by angle which is present at x[1] in increasing order CCW

    sorted_indices = [pc[2] for pc in polar_coords] 
    sorted_polar = [(pc[0], pc[1]) for pc in polar_coords]
    return sorted_indices, sorted_polar


def compute_coverage_angles(sorted_polar, theta_vals_sorted):

    n = len(sorted_polar)
    alphas = [p[1] for p in sorted_polar]
    epsilons = np.zeros(n)

    for i in range(n):
        i_next = (i + 1) % n
        alpha_i = alphas[i]
        alpha_next = alphas[i_next]
        theta_i = theta_vals_sorted[i]
        theta_next = theta_vals_sorted[i_next]

        diff = alpha_next - alpha_i
        # for handle wrap   
        if diff > np.pi:
            diff -= 2.0 * np.pi
        elif diff < -np.pi:
            diff += 2.0 * np.pi

        epsilons[i] = diff - 0.5 * (theta_i + theta_next)

    return epsilons


def compute_group_occupied_angle(theta_vals_sorted, epsilons):
    """
    θ_G = Σ |θ_i| + Σ ε_i (only negative epsilons).
    Inputs must be in the SAME sorted order (consistent with epsilons).
    """
    theta_sum = sum(abs(t) for t in theta_vals_sorted)
    eps_sum = sum(e for e in epsilons if e < 0.0)
    theta_G = theta_sum + eps_sum

    if DEBUG_LOG:
        print(f"[DEBUG] theta_vals_sorted (deg): {np.degrees(theta_vals_sorted)}")
        print(f"[DEBUG] epsilons (deg): {np.degrees(epsilons)}")
        print(f"[DEBUG] theta_G (deg): {np.degrees(theta_G)}")

    return theta_G


def compute_inter_collision_velocity(pursuer_states, i, R_o, R_b):
    """
    Compute the inter-collision avoidance gradient-based velocity term w_i.

    Assumes position is at indices [6], [7].
    """
    p_i = np.array(pursuer_states[i][6:8])
    w_i = np.zeros(2)

    for j, state in enumerate(pursuer_states):
        if i == j:
            continue
        p_j = np.array(state[6:8])
        d_ij = np.linalg.norm(p_i - p_j)

        if d_ij <= R_o or d_ij > R_b:
            continue

        grad_U_ij = (
            2.0 * ((d_ij**2 - R_b**2) / (d_ij**2 - R_o**2 + 1e-8))
            * ((p_i - p_j) / (d_ij**2 + 1e-8))
        )
        w_i -= grad_U_ij

    return w_i


def compute_polygon_maintenance_gradient(pursuer_states, i, R_c, R_f):
    """
    Compute polygon maintenance gradient s_i (distance-regularization) for pursuer i.

    Uses the same position slice [6:8] for all pursuers.
    """
    p_i = np.array(pursuer_states[i][6:8])
    s_i = np.zeros(2)

    for j, state in enumerate(pursuer_states):
        if i == j:
            continue
        p_j = np.array(state[6:8])
        d_ij = np.linalg.norm(p_i - p_j)

        if d_ij <= R_c or d_ij > R_f:
            continue

        grad_Q_ij = (
            2.0 * ((d_ij**2 - R_f**2) / (d_ij**2 - R_c**2 + 1e-8))
            * ((p_i - p_j) / (d_ij**2 + 1e-8))
        )
        s_i -= grad_Q_ij

    return s_i

def check_capture(pursuer_states, evader_state, d_c):
    """
    Returns whether any pursuer is within capture distance of the evader.
    """
    e_pos = evader_state[6:8]
    min_dist = float("inf")
    capture_idx = None

    for i, s in enumerate(pursuer_states):
        d = np.linalg.norm(s[6:8] - e_pos)
        if d < min_dist:
            min_dist = d
            capture_idx = i

    captured = (min_dist <= d_c)
    return captured, capture_idx, min_dist

class ApolloniusTradeoffController:
    def __init__(self, desired_capture_distance=1.0):
        self.beta_history = {}  # optional logging
        self.r_d = desired_capture_distance

    def compute_beta_coefficient(self, i, epsilons, theta_sorted, sorted_polar):
        """
        Compute beta_i as in Fang et al. (Eq. 33-35 style).
        """
        n = len(sorted_polar)
        i_next = (i + 1) % n
        i_prev = (i - 1) % n

        r_i, _ = sorted_polar[i]
        r_next, _ = sorted_polar[i_next]
        r_prev, _ = sorted_polar[i_prev]

        eps_i = epsilons[i]
        eps_prev = epsilons[i_prev]

        theta_i = theta_sorted[i]
        theta_prev = theta_sorted[i_prev]
        theta_next = theta_sorted[i_next]

        denom = 4.0 * np.pi - theta_next + theta_prev
        if denom <= 1e-8:
            denom = 1e-8
        delta_i = (2.0 * abs(eps_i - eps_prev)) / denom

        sum_r = r_i + r_next + r_prev
        if sum_r <= 1e-8:
            sum_r = 1e-8
        ratio = r_i / sum_r
        exponent = 0.609  # ~log_3(2)
        gamma_i = np.sin(np.pi * (ratio ** exponent))

        beta_i = (np.pi / 2.0) * (1.0 - np.exp(-delta_i * gamma_i))

        if DEBUG_LOG:
            print(f"[BETA] i={i}, delta_i={delta_i:.3f}, gamma_i={gamma_i:.3f}, beta_i={beta_i:.3f}")
        return beta_i

    def compute_tradeoff_velocities(self, sorted_polar, epsilons, V_list_sorted,
                                    theta_sorted, pursuers_sorted,
                                    R_o, R_b, R_c, R_f):
        """
        Compute velocity commands for pursuers in sorted order.
        Returns v_commands_sorted in same order as sorted_polar.
        """
        n = len(sorted_polar)
        v_commands_sorted = []

        rs = [p[0] for p in sorted_polar]

        for i in range(n):
            r_i, alpha_i = sorted_polar[i]
            V_i = V_list_sorted[i]

            beta_i = self.compute_beta_coefficient(i, epsilons, theta_sorted, sorted_polar)
            self.beta_history.setdefault(i, []).append(beta_i)

            i_next = (i + 1) % n
            i_prev = (i - 1) % n
            eps_diff = epsilons[i_next] - epsilons[i_prev]

            # k_i, h_i (tradeoff between surrounding and hunting)
            if abs(eps_diff) > 1e-6 and beta_i > 1e-6:
                k_i = V_i * np.sin(beta_i) 
            else:
                k_i = 0.0

            h_i = V_i * np.cos(beta_i) 

            # Surrounding component
            alpha_dot = k_i * eps_diff
            v_is = alpha_dot * r_i * np.array([-np.sin(alpha_i), np.cos(alpha_i)])

            # Hunting component (towards evader)
            v_ih = -h_i * r_i * np.array([np.cos(alpha_i), np.sin(alpha_i)])

            # Collision avoidance & polygon maintenance
            w_i = compute_inter_collision_velocity(pursuers_sorted, i, R_o, R_b)
            s_i = compute_polygon_maintenance_gradient(pursuers_sorted, i, R_c, R_f)

            b = 0.1
            grad = s_i + w_i
            if np.linalg.norm(grad) > 1e-6:
                dir_grad = grad / np.linalg.norm(grad)
            else:
                dir_grad = np.zeros(2)

            v_im = (np.linalg.norm(v_is + v_ih) + b) * dir_grad
            
            v_total = v_is + v_ih + v_im
            norm_v = np.linalg.norm(v_total)
            if norm_v > 1e-6:
                v_total = V_i * v_total / norm_v

            v_commands_sorted.append(v_total)
            EP = -np.array([np.cos(alpha_i), np.sin(alpha_i)])  # unit vector P->E
            radial_component = np.dot(v_total, EP)              # projection toward evader

            if DEBUG_LOG:
                print(f"[P{i}] radial_component={radial_component:.3f}")
                print(f"[PURS {i}] alpha={alpha_i:.2f}, v_is={v_is}, v_ih={v_ih}, v_im={v_im}, v_total={v_total}")

        return v_commands_sorted

    def compute_tradeoff_command(self, pursuer_states, evader_state, V_list,
                                 theta_vals, R_o, R_b):
        """
        High-level function to compute pursuer velocity commands in original index order.

        pursuer_states: list of raw states
        evader_state: evader state
        V_list: list of speeds (same order as pursuer_states)
        theta_vals: list of θ_i in original order
        """
        n = len(pursuer_states)
        if n == 0:
            return []

        # Sort by angle around evader
        sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, evader_state)

        # Reorder all arrays to sorted order
        theta_sorted = [theta_vals[idx] for idx in sorted_indices]
        pursuers_sorted = [pursuer_states[idx] for idx in sorted_indices]
        V_list_sorted = [V_list[idx] for idx in sorted_indices]

        # Compute coverage gaps
        epsilons = compute_coverage_angles(sorted_polar, theta_sorted)

        # Compute velocities in sorted order
        v_cmds_sorted = self.compute_tradeoff_velocities(
            sorted_polar, epsilons, V_list_sorted, theta_sorted,
            pursuers_sorted, R_o, R_b, R_c=3.0, R_f=5.0
        )

        # Map back to original indexing
        v_cmds = [None] * n
        for local_i, v_cmd in enumerate(v_cmds_sorted):
            orig_idx = sorted_indices[local_i]
            v_cmds[orig_idx] = v_cmd

        return v_cmds
