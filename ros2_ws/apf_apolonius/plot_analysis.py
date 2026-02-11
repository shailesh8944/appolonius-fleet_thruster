import numpy as np
import matplotlib.pyplot as plt
import os
from typing import List, Dict, Tuple, Any
from module_kinematics import ssa

def plot_pursuer_evader_distances(states_pursuer1, states_pursuer2, states_pursuer3,
                                 states_evader, plots_dir, time_step=0.1):
    """Plot distance between each pursuer and the evader over time and save figure"""
    pursuer_states = [np.array(states_pursuer1), np.array(states_pursuer2), np.array(states_pursuer3)]
    evader_states = np.array(states_evader)
    
    # Align all series to the shortest available length to avoid shape issues
    min_len = min([s.shape[0] for s in pursuer_states + [evader_states]])
    time = np.arange(min_len) * time_step
    evader_xy = evader_states[:min_len, 6:8]
    
    colors = ['b', 'g', 'r']
    plt.figure(figsize=(12, 6))
    for idx, (state, color) in enumerate(zip(pursuer_states, colors), start=1):
        pursuer_xy = state[:min_len, 6:8]
        distances = np.linalg.norm(pursuer_xy - evader_xy, axis=1)
        plt.plot(time, distances, color=color, label=f'Pursuer {idx}')
    
    # Capture radius threshold (1 m) as a horizontal reference line
    plt.axhline(1.0, color='k', linestyle='--', linewidth=1.5, label='Capture radius (1 m)')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Distance to Evader (m)')
    plt.title('Pursuer-Evader Separation')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'pursuer_evader_distances.png'))
    plt.close()

def plot_evader_strategy_diagnostics(diag_history: List[Dict[str, Any]], plots_dir: str, time_step: float = 0.1,
                                     evader_heading: np.ndarray = None):
    """Generate diagnostic plots to validate Ramana & Kothari evader strategy."""
    if not diag_history:
        print("No evader diagnostics available to plot.")
        return

    os.makedirs(plots_dir, exist_ok=True)
    times = np.array([d.get('time', idx * time_step) for idx, d in enumerate(diag_history)])

    # Collect all unique pair labels across the history to avoid mismatched lengths
    all_pairs = set()
    for d in diag_history:
        for tp in d.get('theta_pairs', []):
            all_pairs.add(tuple(tp['pair']))
    if not all_pairs:
        print("No theta pair data available to plot.")
        return

    theta_traces: Dict[Tuple[int, int], List[float]] = {pair: [] for pair in sorted(all_pairs)}
    active_pairs, theta_active = [], []
    psi_1, psi_2, psi_sel = [], [], []
    dtheta_1, dtheta_2 = [], []
    phi_i, phi_j = [], []
    r_i, r_j = [], []
    active_pair_idx = []

    for diag in diag_history:
        pair_map = {tuple(tp['pair']): tp['theta'] for tp in diag['theta_pairs']}
        for pair in theta_traces:
            theta_traces[pair].append(pair_map.get(pair, np.nan))

        active_pairs.append(tuple(diag['active_pair']))
        theta_active.append(diag['theta_active'])
        psi_1.append(diag['psi_candidate'])
        psi_2.append(diag['psi_candidate_2'])
        psi_sel.append(diag['psi_selected'])
        dtheta_1.append(diag['dtheta_dt_candidate'])
        dtheta_2.append(diag['dtheta_dt_candidate_2'])
        phi_i.append(diag['phi_i'])
        phi_j.append(diag['phi_j'])
        r_i.append(diag['r_i'])
        r_j.append(diag['r_j'])
        active_pair_idx.append(diag.get('active_pair_idx', 0))

    # Ensure every series aligns with the time vector length
    n = len(times)
    theta_traces = {pair: np.array(vals, dtype=float)[:n] for pair, vals in theta_traces.items()}
    theta_active = np.array(theta_active, dtype=float)[:n]
    psi_1 = np.array(psi_1, dtype=float)[:n]
    psi_2 = np.array(psi_2, dtype=float)[:n]
    psi_sel = np.array(psi_sel, dtype=float)[:n]
    dtheta_1 = np.array(dtheta_1, dtype=float)[:n]
    dtheta_2 = np.array(dtheta_2, dtype=float)[:n]
    phi_i = np.array(phi_i, dtype=float)[:n]
    phi_j = np.array(phi_j, dtype=float)[:n]
    r_i = np.array(r_i, dtype=float)[:n]
    r_j = np.array(r_j, dtype=float)[:n]
    active_pair_idx = np.array(active_pair_idx, dtype=int)[:n]

    def wrap_arr(arr):
        """Wrap each angle to [-pi, pi] using ssa."""
        return np.array([ssa(val) for val in arr], dtype=float)

    psi_1_wrapped = wrap_arr(psi_1)
    psi_2_wrapped = wrap_arr(psi_2)
    psi_sel_wrapped = wrap_arr(psi_sel)
    phi_i_wrapped = wrap_arr(phi_i)
    phi_j_wrapped = wrap_arr(phi_j)

    # Optional actual evader heading; trim to match available timestamps
    psi_actual_wrapped = None
    times_for_actual = None
    if evader_heading is not None:
        heading_array = np.asarray(evader_heading, dtype=float)
        if heading_array.size > 0:
            psi_actual_wrapped = wrap_arr(heading_array)
            min_len = min(len(times), len(psi_actual_wrapped))
            times_for_actual = times[:min_len]
            psi_actual_wrapped = psi_actual_wrapped[:min_len]
        else:
            print("Evader heading provided but empty; skipping actual heading plot.")

    # Dashboard of core diagnostics
    fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

    # Row 1: theta_ij for all pairs, highlight active
    for pair, vals in theta_traces.items():
        axes[0].plot(times, np.array(vals), label=f"θ{pair[0]}{pair[1]}")
    axes[0].scatter(times, theta_active, color='k', s=12, label='Active θ_ij')
    axes[0].set_ylabel('θ_ij [rad]')
    axes[0].grid(True)
    axes[0].legend(ncol=3)

    # Row 2: psi candidates and selected

    # Distinct styles to avoid overlap confusion
    axes[1].plot(times, psi_1_wrapped, color='#1f77b4', linestyle='-', linewidth=1.5,
                 marker='o', markersize=3, markevery=max(1, len(times)//40), label='ψ candidate 1')
    axes[1].plot(times, psi_2_wrapped, color='#ff7f0e', linestyle='--', linewidth=1.5,
                 marker='s', markersize=3, markevery=max(1, len(times)//35), label='ψ candidate 2')
    axes[1].plot(times, psi_sel_wrapped, color='#8c564b', linestyle='-', linewidth=2.5,
                 marker='x', markersize=4, markevery=max(1, len(times)//30), label='ψ selected')
    if psi_actual_wrapped is not None and times_for_actual is not None:
        axes[1].plot(times_for_actual, psi_actual_wrapped, color='#9467bd', linestyle='-.', linewidth=2.2,
                     marker='^', markersize=4, markevery=max(1, len(times)//28), label='ψ evader (actual)', zorder=5)
    axes[1].set_ylabel('ψ_e [rad]')
    axes[1].grid(True)
    axes[1].legend()

    # Row 3: dtheta/dt for both candidates
    axes[2].plot(times, dtheta_1, 'b-', label='dθ/dt (ψ1)')
    axes[2].plot(times, dtheta_2, 'g--', label='dθ/dt (ψ2)')
    axes[2].set_ylabel('dθ/dt [arb]')
    axes[2].grid(True)
    axes[2].legend()

    # Row 4: distances to active pair
    axes[3].plot(times, r_i, 'm-', label='r_i')
    axes[3].plot(times, r_j, 'c--', label='r_j')
    axes[3].axhline(1.0, color='k', linestyle=':', label='Capture radius')
    axes[3].set_ylabel('Range [m]')
    axes[3].grid(True)
    axes[3].legend()

    # Row 5: LOS angles
    axes[4].plot(times, phi_i_wrapped, 'm-', label='φ_i')
    axes[4].plot(times, phi_j_wrapped, 'c--', label='φ_j')
    axes[4].set_ylabel('LOS [rad]')
    axes[4].set_xlabel('Time [s]')
    axes[4].grid(True)
    axes[4].legend()

    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, 'evader_strategy_dashboard.png'))
    plt.close(fig)

    # Scatter: θ_active vs ψ_selected
    plt.figure(figsize=(8, 6))
    plt.scatter(psi_sel_wrapped, theta_active, c=times, cmap='viridis', s=30)
    plt.colorbar(label='Time [s]')
    plt.xlabel('ψ_e selected [rad]')
    plt.ylabel('Active θ_ij [rad]')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'evader_theta_vs_heading.png'))
    plt.close()

    # Active pair index vs time
    plt.figure(figsize=(8, 4))
    plt.step(times, active_pair_idx, where='post')
    plt.xlabel('Time [s]')
    plt.ylabel('Active pair index')
    plt.yticks(range(len(all_pairs)))
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'evader_active_pair_index.png'))
    plt.close()

    # Quick vector snapshots to show escape direction
    snap_indices = np.linspace(0, len(diag_history) - 1, num=min(3, len(diag_history)), dtype=int)
    fig, axes = plt.subplots(1, len(snap_indices), figsize=(6 * len(snap_indices), 6))
    if len(snap_indices) == 1:
        axes = [axes]
    for ax, idx in zip(axes, snap_indices):
        d = diag_history[idx]
        ev = np.array(d['evader_pos'])
        pursuers = d['pursuer_positions']
        ax.scatter(ev[0], ev[1], c='r', label='Evader')
        for p_i, pos in enumerate(pursuers):
            ax.scatter(pos[0], pos[1], c='b', marker='x', label='Pursuer' if p_i == 0 else None)
            ax.plot([ev[0], pos[0]], [ev[1], pos[1]], 'k:', linewidth=0.8)
        # Arrows for ψ candidates and selected
        for angle, color, lbl in [
            (d['psi_candidate'], "#1fb44c", 'ψ1'),
            (d['psi_candidate_2'], "#120eff", 'ψ2'),
            (d['psi_selected'], "#658c4b", 'ψ sel')
        ]:
            ax.arrow(ev[0], ev[1], np.cos(angle), np.sin(angle),
                     head_width=0.5, length_includes_head=True, color=color, label=lbl)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title(f"t = {times[idx]:.1f}s")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=4)
    fig.tight_layout()
    fig.savefig(os.path.join(plots_dir, 'evader_escape_vectors.png'))
    plt.close(fig)

def generate_evader_strategy_plots(evader_diag_history, plots_dir="plots", time_step=0.1, evader_heading=None):
    """Wrapper to produce all evader-specific diagnostic plots."""
    plot_evader_strategy_diagnostics(evader_diag_history, plots_dir, time_step=time_step,
                                     evader_heading=evader_heading)

def plot_ship_analysis(states, commanded_rudders, commanded_props, name, save_prefix, plots_dir,
                       time_step=0.1):
    """
    Enhanced analysis plots for ship motion with proper directory handling
    """
    # Create time array
    if time_step <= 0:
        time_step = 0.1
    if commanded_props is None:
        commanded_props = []
    if commanded_rudders is None:
        commanded_rudders = []

    n_ref = len(commanded_props) if len(commanded_props) > 0 else len(commanded_rudders)
    if n_ref == 0:
        return
    time = np.arange(n_ref) * time_step
    states = states[:n_ref]
    
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
    plt.plot(time, states[:, 5], 'g-', label='Yaw rate (r)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw rate (rad/s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f'{save_prefix}_velocity_analysis.png'))
    plt.close()
    
    # Plot 2: Control only (skip if no rudder data)
    if len(commanded_rudders) > 0:
        plt.figure(figsize=(12, 4))
        plt.plot(time, commanded_rudders, 'b-', label='Commanded Rudder')
        m = states[:, 12] * 180 / np.pi  # Convert radians to degrees
        plt.plot(time, m, 'r--', label='Actual Rudder')
        plt.xlabel('Time (s)')
        plt.ylabel('Rudder Angle (deg)')
        plt.title(f'{name} - Rudder Analysis')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plots_dir, f'{save_prefix}_control_analysis.png'))
        plt.close()
    
    # Plot 3: Position and Trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(states[:, 6], states[:, 7], 'b-', label='Trajectory')
    plt.plot(states[0, 6], states[0, 7], 'go', label='Start')
    plt.plot(states[-1, 6], states[-1, 7], 'ro', label='End')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title(f'{name} - Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(plots_dir, f'{save_prefix}_trajectory.png'))
    plt.close()
    
    # Print analysis information
    print(f"\n=== {name} Analysis ===")
    print(f"Initial position: ({states[0, 6]:.1f}, {states[0, 7]:.1f})")
    print(f"Final position: ({states[-1, 6]:.1f}, {states[-1, 7]:.1f})")
    print(f"Average velocity: {np.mean(np.sqrt(states[:, 0]**2 + states[:, 1]**2)):.2f} m/s")
    if len(commanded_props) > 0:
        print(f"Average propeller: {np.mean(commanded_props):.1f} RPM")
    print(f"Distance traveled: {np.sum(np.sqrt(np.diff(states[:, 6])**2 + np.diff(states[:, 7])**2)):.2f} m")

def generate_analysis_plots(states_pursuer1, states_pursuer2, states_pursuer3,
                            states_evader, commanded_rudders_p1, commanded_rudders_p2,
                            commanded_rudders_p3, commanded_rudders_e,
                            commanded_props_p1, commanded_props_p2,
                            commanded_props_p3, commanded_props_e, plots_dir="plots",
                            time_step=0.1,
                            port_act_p1=None, stbd_act_p1=None,
                            port_act_p2=None, stbd_act_p2=None,
                            port_act_p3=None, stbd_act_p3=None,
                            port_act_e=None, stbd_act_e=None):
    """Generate and save all analysis plots in the specified directory"""
    
    # Ensure plots directory exists
    os.makedirs(plots_dir, exist_ok=True)
    print(f"\nSaving plots to: {plots_dir}")
    
    # Combined trajectory plot
    plt.figure(figsize=(12, 10))
    plt.plot(states_pursuer1[:, 6], states_pursuer1[:, 7], 'b-', label='Pursuer 1')
    plt.plot(states_pursuer2[:, 6], states_pursuer2[:, 7], 'g-', label='Pursuer 2')
    plt.plot(states_pursuer3[:, 6], states_pursuer3[:, 7], 'r-', label='Pursuer 3')
    plt.plot(states_evader[:, 6], states_evader[:, 7], 'k-', label='Evader')
    
    # Add start positions
    plt.plot(states_pursuer1[0, 6], states_pursuer1[0, 7], 'bo', label='P1 Start')
    plt.plot(states_pursuer2[0, 6], states_pursuer2[0, 7], 'go', label='P2 Start')
    plt.plot(states_pursuer3[0, 6], states_pursuer3[0, 7], 'ro', label='P3 Start')
    plt.plot(states_evader[0, 6], states_evader[0, 7], 'ko', label='E Start')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Combined Trajectories')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.savefig(os.path.join(plots_dir, 'combined_trajectories.png'))
    plt.close()
    
    # Individual analysis plots
    states_list = [states_pursuer1, states_pursuer2, states_pursuer3, states_evader]
    rudders_list = [commanded_rudders_p1, commanded_rudders_p2, 
                    commanded_rudders_p3, commanded_rudders_e]
    props_list = [commanded_props_p1, commanded_props_p2, 
                  commanded_props_p3, commanded_props_e]
    names = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3', 'Evader']
    prefixes = ['pursuer1', 'pursuer2', 'pursuer3', 'evader']
    
    for states, rudders, props, name, prefix in zip(states_list, rudders_list,
                                                    props_list, names, prefixes):
        states = np.array(states)
        rudders = None if rudders is None else np.array(rudders)
        props = None if props is None else np.array(props)
        
        print(f"\nGenerating plots for {name}")
        plot_ship_analysis(states, rudders, props, name, prefix, plots_dir,
                           time_step=time_step)

    def _plot_thrusters(port_act, stbd_act, title, filename, port_label, stbd_label):
        if port_act is None or stbd_act is None:
            return
        port_act = np.array(port_act, dtype=float)
        stbd_act = np.array(stbd_act, dtype=float)
        n_ref = min(len(port_act), len(stbd_act))
        if n_ref <= 0:
            return
        time = np.arange(n_ref) * time_step
        plt.figure(figsize=(12, 4))
        plt.plot(time, port_act[:n_ref], 'b-', label=port_label)
        plt.plot(time, stbd_act[:n_ref], 'r--', label=stbd_label)
        plt.xlabel('Time (s)')
        plt.ylabel('Actuator Command')
        plt.title(title)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plots_dir, filename))
        plt.close()

    # Thruster actuator plots
    _plot_thrusters(port_act_p1, stbd_act_p1,
                    "Pursuer 1 Thruster Commands", "pursuer1_thruster_commands.png",
                    "P1 Port Actuator", "P1 Stbd Actuator")
    _plot_thrusters(port_act_p2, stbd_act_p2,
                    "Pursuer 2 Thruster Commands", "pursuer2_thruster_commands.png",
                    "P2 Port Actuator", "P2 Stbd Actuator")
    _plot_thrusters(port_act_p3, stbd_act_p3,
                    "Pursuer 3 Thruster Commands", "pursuer3_thruster_commands.png",
                    "P3 Port Actuator", "P3 Stbd Actuator")
    _plot_thrusters(port_act_e, stbd_act_e,
                    "Evader Thruster Commands", "evader_thruster_commands.png",
                    "Evader Port Actuator", "Evader Stbd Actuator")
    
    # Plot distances between each pursuer and the evader
    plot_pursuer_evader_distances(states_pursuer1, states_pursuer2, states_pursuer3,
                                  states_evader, plots_dir)
