import numpy as np

# ---------------------------
# Angle utilities
# ---------------------------

def ssa(ang, deg=False):
    """
    Smallest Signed Angle (SSA) function to wrap angle to [-180, 180] degrees or 
    [-pi, pi] radians.
    """
    if deg:
        ang = (ang + 180) % 360 - 180
    else:
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
    return ang


# ---------------------------
# Geometry around evader
# ---------------------------

def compute_local_polar(pursuer_pos, evader_pos):
    """
    Polar coordinates of pursuer w.r.t. evader (origin at evader).
    pursuer_pos, evader_pos: (x, y)
    Returns:
        r   : distance evader -> pursuer
        alpha : angle in [0, 2π)
    """
    dx = pursuer_pos[0] - evader_pos[0]
    dy = pursuer_pos[1] - evader_pos[1]
    r = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx)
    alpha = (alpha + 2*np.pi) % (2*np.pi)
    return r, alpha


def sort_pursuers_by_angle(pursuer_states, evader_state):
    """
    Sort pursuers by angle α_i around evader (evader at origin).
    Assumes positions at indices [6:8] = (x, y).
    """
    evader_pos = np.array([evader_state[6], evader_state[7]])
    polar_coords = []
    for i, state in enumerate(pursuer_states):
        px, py = state[6], state[7]
        r_i, alpha_i = compute_local_polar((px, py), evader_pos)
        polar_coords.append((r_i, alpha_i, i))

    # Sort by alpha
    polar_coords.sort(key=lambda x: x[1])

    sorted_indices = [pc[2] for pc in polar_coords]
    sorted_polar = [(pc[0], pc[1]) for pc in polar_coords]  # (r, alpha) in sorted order
    return sorted_indices, sorted_polar


def compute_coverage_angles(sorted_polar, theta_sorted):
    """
    Compute ε_i between consecutive pursuers in sorted order.

    sorted_polar : list of (r_i, alpha_i) already sorted by alpha_i
    theta_sorted : θ_i in the SAME sorted order
    """
    n = len(sorted_polar)
    alphas = [p[1] for p in sorted_polar]
    epsilons = np.zeros(n)

    for i in range(n):
        i_next = (i + 1) % n
        alpha_i = alphas[i]
        alpha_next = alphas[i_next]
        theta_i = theta_sorted[i]
        theta_next = theta_sorted[i_next]

        # Circular forward difference in [0, 2π)
        diff = (alpha_next - alpha_i) % (2*np.pi)
        epsilons[i] = diff - (theta_i + theta_next) / 2.0

    return epsilons


def compute_group_occupied_angle(theta_vals, epsilons):
    """
    θ_G = Σ |θ_i| + Σ ε_i   for ε_i < 0
    """
    theta_sum = 0.0
    for theta in theta_vals:
        theta_sum += abs(theta)

    eps_sum = 0.0
    for eps in epsilons:
        if eps < 0:
            eps_sum += eps

    theta_G = theta_sum + eps_sum

    print(f"Debug - theta_vals (deg): {np.degrees(theta_vals)}")
    print(f"Debug - epsilons (deg): {np.degrees(epsilons)}")
    print(f"Debug - theta_G (deg): {np.degrees(theta_G)}")

    return theta_G


# ---------------------------
# Gradients for polygon & collision
# ---------------------------

def compute_polygon_maintenance_gradient(pursuer_states, i, R_c, R_f):
    """
    Compute polygon maintenance gradient (s_i) for pursuer i.
    Uses positions [6:8] = (x, y).
    Only neighbors with R_c < d_ij <= R_f are counted.
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

        # Avoid division by zero with small eps
        eps = 1e-8
        grad_Q_ij = (
            2.0 * ((d_ij**2 - R_f**2) / (d_ij**2 - R_c**2 + eps)) *
            ((p_i - p_j) / (d_ij**2 + eps))
        )
        s_i -= grad_Q_ij

    return s_i


def compute_inter_collision_velocity(pursuer_states, i, R_o, R_b):
    """
    Inter-collision avoidance velocity for pursuer i.
    Uses positions [6:8] = (x, y).
    Only neighbors with R_o < d_ij <= R_b are counted.
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

        eps = 1e-8
        grad_U_ij = (
            2.0 * ((d_ij**2 - R_b**2) / (d_ij**2 - R_o**2 + eps)) *
            ((p_i - p_j) / (d_ij**2 + eps))
        )
        w_i -= grad_U_ij

    return w_i


# ---------------------------
# Numerical derivative helper
# ---------------------------

def numerical_partial_derivative(func, x, idx, delta=1e-5):
    """
    Central difference ∂func/∂x[idx] at vector x.
    """
    x1 = np.array(x, dtype=float)
    x2 = np.array(x, dtype=float)
    x1[idx] += delta
    x2[idx] -= delta
    return (func(x1) - func(x2)) / (2 * delta)


# ---------------------------
# Apollonius radius & ∂ε/∂r
# ---------------------------

def radius_encircle(pursuer_states, evader_state, theta_vals):
    """
    Compute Apollonius encircle radius and ∂ε_i/∂r_i for each pursuer.

    Returns:
        radii          : list radius_i (original indexing)
        dtheta_dr_list : list ∂ε_i/∂r_i (original indexing)
    """
    evader_speed = evader_state[0]
    n = len(pursuer_states)

    radii = [0.0] * n
    dtheta_dr_list = [0.0] * n

    # Sorted geometry around evader
    sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, evader_state)
    # θ values in sorted order:
    theta_sorted = [theta_vals[idx] for idx in sorted_indices]
    # r variables in sorted order:
    r_vars = [p[0] for p in sorted_polar]

    # For each pursuer in sorted order
    for sorted_idx, orig_idx in enumerate(sorted_indices):
        state = pursuer_states[orig_idx]
        pursuer_speed = state[0]

        # distance from evader (consistent with positions [6:8])
        r_i = np.linalg.norm(state[6:8] - evader_state[6:8])

        lam = pursuer_speed / evader_speed if evader_speed != 0 else 1e-8
        if np.isclose(lam, 0.0):
            radius = 0.0
        else:
            radius = ((1.0 - lam**2) / lam) * r_i
        radii[orig_idx] = radius

        # Wrapper for numerical derivative of ε_i wrt r_i
        def coverage_angle_wrapper(v, target_idx=sorted_idx):
            # v is the list of r_j in sorted order
            polar = [(v[j], sorted_polar[j][1]) for j in range(n)]
            epsilons = compute_coverage_angles(polar, theta_sorted)
            return epsilons[target_idx]

        dtheta_dr_i = numerical_partial_derivative(coverage_angle_wrapper, r_vars, sorted_idx)
        dtheta_dr_list[orig_idx] = dtheta_dr_i

    return radii, dtheta_dr_list


# ---------------------------
# Evader heading computation
# ---------------------------

def compute_evader_heading(pursuer_states, evader_state, theta_vals, dtheta_dr_list):
    """
    Compute evader heading ψ_e based on the largest uncovered gap.

    1. Sort pursuers by angle around evader.
    2. Compute ε_i between consecutive pursuers.
    3. Choose pair producing max ε_i (largest gap).
    4. Use ∂ε/∂r of that pair in the condition:
         w_i sin(ψ_e - φ_i) + w_j sin(ψ_e - φ_j) = 0
       ⇒ ψ_e = atan2( w_i sin φ_i + w_j sin φ_j,
                      w_i cos φ_i + w_j cos φ_j )
    """
    n = len(pursuer_states)
    if n < 2:
        return 0.0

    e_pos = evader_state[6:8]

    # Sort by angle around evader
    sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, evader_state)
    theta_sorted = [theta_vals[idx] for idx in sorted_indices]
    epsilons = compute_coverage_angles(sorted_polar, theta_sorted)

    # Largest gap
    gap_idx = int(np.argmax(epsilons))
    idx_i = sorted_indices[gap_idx]
    idx_j = sorted_indices[(gap_idx + 1) % n]

    def los_angle(p_idx):
        pursuer_pos = pursuer_states[p_idx][6:8]
        phi = np.arctan2(e_pos[1] - pursuer_pos[1], e_pos[0] - pursuer_pos[0])
        # Put LOS in [0, 2π)
        phi = (phi + 2*np.pi) % (2*np.pi)
        return phi

    phi_i = los_angle(idx_i)
    phi_j = los_angle(idx_j)

    # Weights from ∂ε/∂r
    weights = np.array(dtheta_dr_list, dtype=float)
    w_i = weights[idx_i]
    w_j = weights[idx_j]

    # If both are ~0, just go through midpoint of gap
    if np.isclose(w_i, 0.0) and np.isclose(w_j, 0.0):
        alpha_i = sorted_polar[gap_idx][1]
        alpha_j = sorted_polar[(gap_idx + 1) % n][1]
        psi_e = 0.5 * (alpha_i + alpha_j)
        psi_e = (psi_e + 2*np.pi) % (2*np.pi)
        return ssa(psi_e)

    numerator = w_i * np.sin(phi_i) + w_j * np.sin(phi_j)
    denominator = w_i * np.cos(phi_i) + w_j * np.cos(phi_j)
    psi_e = np.arctan2(numerator, denominator)  # [-π, π]
    psi_e = (psi_e + 2*np.pi) % (2*np.pi)       # [0, 2π)
    psi_e_ssa = ssa(psi_e)                      # final controller form [-π, π]

    print(
        f"Evader heading via pair ({idx_i}, {idx_j}) -> psi_e (rad): {psi_e_ssa:.3f}, "
        f"gap (deg): {np.degrees(epsilons[gap_idx]):.2f}"
    )
    return psi_e_ssa


# ---------------------------
# Apollonius-based tradeoff controller for pursuers
# ---------------------------

class ApolloniusTradeoffController:
    def __init__(self, desired_capture_distance=2.0):
        self.beta_history = {}   # will initialize dynamically
        self.r_d = desired_capture_distance

    def compute_beta_coefficient(self, i, epsilons, theta_sorted, sorted_polar):
        """
        i is index in SORTED order.
        """
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
        delta_eps = eps_i - eps_prev

        # Occupied angles for i and i+1 in sorted order
        theta_i = theta_sorted[i]
        theta_next = theta_sorted[i_next]

        denom = 4.0 * np.pi - theta_next + theta_i
        if denom <= 1e-8:
            denom = 1e-8
        delta_i = (2.0 * abs(delta_eps)) / denom

        sum_r = r_i + r_next + r_prev
        if sum_r <= 1e-8:
            sum_r = 1e-8

        ratio = r_i / sum_r
        exponent = 0.609  # log3(2) approx
        gamma_i = np.sin(np.pi * (ratio ** exponent))

        beta_i = (np.pi / 2.0) * (1.0 - np.exp(-delta_i * gamma_i))

        print(f"delta_i: {delta_i:.2f}, gamma_i: {gamma_i:.2f}, beta_i: {beta_i:.2f}")
        return beta_i

    def compute_tradeoff_velocities(self, sorted_indices, sorted_polar, epsilons,
                                    V_sorted, theta_sorted, pursuer_states,
                                    R_o, R_b, R_c, R_f):
        """
        Compute velocity commands in SORTED order.
        V_sorted : pursuer speeds in sorted order
        """
        n = len(sorted_polar)
        v_commands_sorted = []

        # Ensure beta_history keys
        for orig_idx in sorted_indices:
            if orig_idx not in self.beta_history:
                self.beta_history[orig_idx] = []

        for k in range(n):
            orig_idx = sorted_indices[k]     # map sorted -> original index
            r_i, alpha_i = sorted_polar[k]

            beta_i = self.compute_beta_coefficient(k, epsilons, theta_sorted, sorted_polar)
            self.beta_history[orig_idx].append(beta_i)

            # Hunting and surrounding velocities
            h_s = 0.008   # hunting coefficent 
            v_ih = h_s * r_i * np.array([np.cos(alpha_i), np.sin(alpha_i)])

            alpha_dot = 0.001 * V_sorted[k] * (
                epsilons[(k + 1) % n] - epsilons[(k - 1) % n]
            )
            v_is = alpha_dot * r_i * np.array([-np.sin(alpha_i), np.cos(alpha_i)])

            # Inter-collision avoidance and polygon maintenance (in ORIGINAL index space)
            w_i = compute_inter_collision_velocity(pursuer_states, orig_idx, R_o, R_b)
            s_i = compute_polygon_maintenance_gradient(pursuer_states, orig_idx, R_c, R_f)

            b = 0.001
            # Use sign of combined gradient, magnitude based on |v_is+v_ih|
            sign_vec = np.sign(s_i + w_i)
            v_im = (np.linalg.norm(v_is + v_ih) + b) * sign_vec

            v_total = v_ih + v_is + 0.2 * v_im

            v_commands_sorted.append(v_total)

            print(f"Pursuer(sorted idx={k}, orig idx={orig_idx}):")
            print(f"  Alpha: {alpha_i:.2f} rad")
            print(f"  Beta: {beta_i:.2f} rad")
            print(f"  v_ih: {v_ih}, v_is: {v_is}, v_im: {v_im}")
            print(f"  Total velocity: {v_total}")

        return v_commands_sorted

    def compute_tradeoff_command(self, pursuer_states, evader_state,
                                 V_list, theta_vals,
                                 d_c, lambda_min, R_p,   # currently unused here
                                 R_o, R_b,
                                 R_c=3.0, R_f=5.0):
        """
        High-level function to compute velocity commands v_i for all pursuers
        in original indexing.
        """
        n = len(pursuer_states)

        # Sort pursuers around evader
        sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, evader_state)

        # Sort theta and speeds to match sorted order
        theta_sorted = [theta_vals[idx] for idx in sorted_indices]
        V_sorted = [V_list[idx] for idx in sorted_indices]

        # Coverage angles ε_i in sorted order
        epsilons = compute_coverage_angles(sorted_polar, theta_sorted)

        # Compute velocities in sorted order
        v_cmds_sorted = self.compute_tradeoff_velocities(
            sorted_indices, sorted_polar, epsilons,
            V_sorted, theta_sorted,
            pursuer_states, R_o, R_b, R_c, R_f
        )

        # Map back to original index order
        v_cmds = [None] * n
        for idx_sorted, orig_idx in enumerate(sorted_indices):
            v_cmds[orig_idx] = v_cmds_sorted[idx_sorted]

        return v_cmds
