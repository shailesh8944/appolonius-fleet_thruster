import numpy as np
from module_kinematics import ssa   # your existing SSA

# =========================
# Small helper: central diff
# =========================
def numerical_partial_derivative(func, x, idx, delta=1e-5):
    """
    Central difference estimate of d func / d x[idx].
    func: function(v) -> scalar
    x: array-like
    idx: index in x
    """
    x1 = np.array(x, dtype=float)
    x2 = np.array(x, dtype=float)
    x1[idx] += delta
    x2[idx] -= delta
    return (func(x1) - func(x2)) / (2.0 * delta)


# ======================================
# 1. Apollonius geometry for all pursuers
# ======================================
def compute_apollonius_geometry(pursuer_states, evader_state):
    """
    Compute Apollonius centre (O_i) and radius (r_i) for each pursuer
    w.r.t a single evader, following Ramana & Kothari.

    Assumes:
        state[0] = surge speed V_p,
        state[6:8] = (x, y) in plane
        evader_state[0] = V_e, evader_state[6:8] = (x_e, y_e)

    Returns:
        lam: speed ratio (Vp / Ve), scalar (using mean pursuer speed)
        centers: list of 2D numpy arrays O_i
        radii:   list of Apollonius radii r_i
    """
    # Speeds
    V_e = float(evader_state[0])
    eps = 1e-8
    V_e_safe = V_e if abs(V_e) > eps else eps

    V_p_list = [float(s[0]) for s in pursuer_states]
    V_p_mean = float(np.mean([abs(v) for v in V_p_list]))
    lam = V_p_mean / V_e_safe

    # Clamp λ into (0,1) for high-speed evader case
    lam = float(np.clip(lam, 1e-4, 0.9999))

    x_e, y_e = float(evader_state[6]), float(evader_state[7])

    centers = []
    radii   = []

    denom = 1.0 - lam**2   # > 0 because lam < 1

    for s in pursuer_states:
        x_p, y_p = float(s[6]), float(s[7])
        d_PE = np.hypot(x_p - x_e, y_p - y_e)  # |P - E|

        # Center O_i [eq. (4)]
        O_x = (x_p - lam**2 * x_e) / denom
        O_y = (y_p - lam**2 * y_e) / denom
        centers.append(np.array([O_x, O_y], dtype=float))

        # Radius r_i [eq. (5)]
        r_i = (lam / denom) * d_PE
        radii.append(float(r_i))

    return lam, centers, radii


# =========================================
# 2. Overlapping angle θ_ij and its dθ/dr_i
# =========================================
def compute_overlapping_angle_and_derivs(i, j, lam, centers, radii):
    """
    For neighbors i, j (in some ordering), compute:
        θ_ij  (overlapping angle between their Apollonius circles)
        ∂θ_ij/∂r_i, ∂θ_ij/∂r_j   (using numerical differentiation)

    Uses Ramana & Kothari eq. (6):
        θ_ij = 2 asin(λ) - acos( (r_i^2 + r_j^2 - (λ d_ij)^2) / (2 r_i r_j) )
    """
    r_i = float(radii[i])
    r_j = float(radii[j])

    O_i = np.asarray(centers[i], dtype=float)
    O_j = np.asarray(centers[j], dtype=float)

    d_ij = float(np.linalg.norm(O_i - O_j))

    lam_safe = float(np.clip(lam, 1e-6, 0.999999))

    def theta_from_r(v):
        """v = [r_i, r_j] -> θ_ij according to eq. (6)."""
        ri, rj = float(v[0]), float(v[1])

        # avoid division issues
        ri = max(ri, 1e-6)
        rj = max(rj, 1e-6)

        # inside acos: A = (ri^2 + rj^2 - (lam*d)^2) / (2 ri rj)
        A = (ri**2 + rj**2 - (lam_safe * d_ij)**2) / (2.0 * ri * rj)
        A = float(np.clip(A, -1.0 + 1e-8, 1.0 - 1e-8))

        term1 = 2.0 * np.arcsin(lam_safe)
        term2 = np.arccos(A)

        return term1 - term2

    # base angle
    theta_ij = theta_from_r([r_i, r_j])

    # numerical partials w.r.t r_i and r_j
    dtheta_dri = numerical_partial_derivative(theta_from_r, [r_i, r_j], idx=0)
    dtheta_drj = numerical_partial_derivative(theta_from_r, [r_i, r_j], idx=1)

    return theta_ij, dtheta_dri, dtheta_drj


# =======================================
# 3. Evader heading ψ_e (Ramana & Kothari)
# =======================================
def compute_evader_heading_ramana(pursuer_states, evader_state, return_debug=False, debug_store=None):
    """
    Implements eq. (8)-(9) of Ramana & Kothari (2017) for evader heading.

    Steps:
      1) Build Apollonius circles (centers and radii).
      2) Sort pursuers around evader in CCW angle.
      3) For each neighboring pair (i,j) in this order, compute overlapping angle θ_ij.
      4) Select pair with *smallest* θ_ij (most vulnerable).
      5) For that pair, compute ∂θ/∂r_i, ∂θ/∂r_j.
      6) Compute ψ_e using eq. (9):
             tan ψ_e = [ (∂θ/∂r_i) sin φ_i + (∂θ/∂r_j) sin φ_j ] /
                       [ (∂θ/∂r_i) cos φ_i + (∂θ/∂r_j) cos φ_j ]
         where φ_i, φ_j are LOS angles of EP_i and EP_j.
      7) Return ψ_e wrapped to [-π, π] via ssa().
    """
    n = len(pursuer_states)
    if n < 2:
        return (0.0, {}) if return_debug else 0.0

    # --- Apollonius geometry ---
    lam, centers, radii = compute_apollonius_geometry(pursuer_states, evader_state)

    # --- Sort pursuers around evader by polar angle ---
    e_pos = np.array(evader_state[6:8], dtype=float)
    angles = []
    for idx, s in enumerate(pursuer_states):
        p_pos = np.array(s[6:8], dtype=float)
        # angle of vector from evader to pursuer (EP_i)
        alpha = np.arctan2(p_pos[1] - e_pos[1], p_pos[0] - e_pos[0])
        # wrap to [0, 2π)
        alpha = (alpha + 2.0 * np.pi) % (2.0 * np.pi)
        angles.append((alpha, idx))

    angles.sort(key=lambda x: x[0])
    sorted_indices = [a[1] for a in angles]

    # --- Compute overlapping angle for each neighboring pair ---
    theta_list = []
    pair_data  = []   # store (i_sorted, j_sorted, theta_ij, dθ/dr_i, dθ/dr_j)

    theta_pairs = []
    for k in range(n):
        i_sorted = sorted_indices[k]
        j_sorted = sorted_indices[(k + 1) % n]

        theta_ij, dtheta_dri, dtheta_drj = compute_overlapping_angle_and_derivs(i_sorted, j_sorted,
                                                 lam, centers, radii)
        theta_list.append(theta_ij)
        pair_data.append((i_sorted, j_sorted, theta_ij, dtheta_dri, dtheta_drj))
        theta_pairs.append({'pair': (i_sorted, j_sorted), 'theta': theta_ij})

    # --- Choose pair with smallest overlapping angle (most critical) ---
    min_idx = int(np.argmin(theta_list))
    i_idx, j_idx, theta_crit, dθ_dri, dθ_drj = pair_data[min_idx]

    # --- LOS angles φ_i, φ_j (angle of EP_i, EP_j) ---
    def los_EP(p_idx):
        p_pos = np.array(pursuer_states[p_idx][6:8], dtype=float)
        vec = p_pos - e_pos       # EP_i
        return np.arctan2(vec[1], vec[0])

    φ_i = los_EP(i_idx)
    φ_j = los_EP(j_idx)

    # --- Compute ψ_e from eq. (9) ---
    num = dθ_dri * np.sin(φ_i) + dθ_drj * np.sin(φ_j)
    den = dθ_dri * np.cos(φ_i) + dθ_drj * np.cos(φ_j)

    ψ_candidate = np.arctan2(num, den)   # one branch of eq. (9)

    # Second possible solution ψ + π
    ψ_candidate_2 = ψ_candidate + np.pi

    # OPTIONAL: choose the one that *minimizes* dθ/dt (more negative).
    # For simplicity, evaluate dθ/dt along both and pick smaller.
    def dtheta_dt_for_ψ(ψ_e):
        """
        Use the expression right before eq. (7),
        but with only the evader's contribution (since we are
        choosing ψ_e to minimize its influence).
        Approx: dθ/dt ≈ (∂θ/∂r_i) * Ve * (-cos(ψ_e - φ_i)) * λ/(1-λ^2)
              + same for j. Up to a scaling, only sign comparison matters.
        """
        # Only relative sign is important; we ignore constants.
        term_i = dθ_dri * (-np.cos(ψ_e - φ_i))
        term_j = dθ_drj * (-np.cos(ψ_e - φ_j))
        return term_i + term_j

    val1 = dtheta_dt_for_ψ(ψ_candidate)
    val2 = dtheta_dt_for_ψ(ψ_candidate_2)

    if val2 < val1:
        ψ_e = ψ_candidate_2
    else:
        ψ_e = ψ_candidate

    # Wrap to [-π, π]
    ψ_e = float(ssa(ψ_e))
    r_i = float(np.linalg.norm(pursuer_states[i_idx][6:8] - e_pos))
    r_j = float(np.linalg.norm(pursuer_states[j_idx][6:8] - e_pos))

    debug = {
        'theta_pairs': theta_pairs,
        'active_pair': (i_idx, j_idx),
        'active_pair_idx': min_idx,
        'theta_active': theta_crit,
        'psi_candidate': float(ssa(ψ_candidate)),
        'psi_candidate_2': float(ssa(ψ_candidate_2)),
        'psi_selected': ψ_e,
        'dtheta_dt_candidate': float(val1),
        'dtheta_dt_candidate_2': float(val2),
        'phi_i': float(φ_i),
        'phi_j': float(φ_j),
        'r_i': r_i,
        'r_j': r_j,
        'evader_pos': e_pos.copy(),
        'pursuer_positions': [np.array(s[6:8], dtype=float) for s in pursuer_states],
        'lambda': lam
    }

    if debug_store is not None:
        debug_store.append(debug)

    # print(f"[EVADER] Critical pair ({i_idx}, {j_idx}), "
    #       f"θ_ij={np.degrees(theta_crit):.2f} deg, "
    #       f"dθ/dr_i={dθ_dri:.3e}, dθ/dr_j={dθ_drj:.3e}, "
    #       f"ψ_e={np.degrees(ψ_e):.2f} deg")

    if return_debug:
        return ψ_e, debug
    return ψ_e
