import casadi as ca

import os
import yaml
from casadi_class_vessel import Vessel
import numpy as np

# Max static thrust (N) per thruster for the simulator PWM fit at act=+1 and thrust_scale=1.
_SIM_ACT1_THRUST_N = 0.39462504 * 9.80665


def _safe_load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _resolve_thruster_model_params(params, base_dir):
    """
    Align NMPC thruster surrogate with simulator configuration.
    Returns: (thrust_scale, thruster_lever, T_act)
    """
    # Defaults from local NMPC config.
    align_with_sim = bool(params.get('align_thruster_with_simulator', True))
    sim_thrust_scale = float(params.get('simulator_thrust_scale', 1.0))
    if align_with_sim:
        thrust_scale = _SIM_ACT1_THRUST_N * sim_thrust_scale
    else:
        thrust_scale = float(params.get('thrust_scale', _SIM_ACT1_THRUST_N * sim_thrust_scale))
    thruster_lever = float(params.get('thruster_lever', 0.21))
    T_act = float(params.get('T_act', 0.2))

    # Prefer simulator thruster geometry/time constants when available.
    thruster_cfg = params.get(
        'thruster_config_path',
        os.path.join(base_dir, '..', '..', 'makara', 'inputs', 'evader', 'thrusters.yml'),
    )
    if not os.path.isabs(thruster_cfg):
        thruster_cfg = os.path.normpath(os.path.join(base_dir, thruster_cfg))

    if os.path.exists(thruster_cfg):
        try:
            thr_data = _safe_load_yaml(thruster_cfg) or {}
            thr_list = thr_data.get('thrusters', [])

            y_vals = []
            t_vals = []
            for thr in thr_list:
                loc = thr.get('thruster_location', [0.0, 0.0, 0.0])
                if len(loc) >= 2:
                    y_vals.append(float(loc[1]))
                t_vals.append(float(thr.get('T_act', T_act)))

            if len(y_vals) >= 2:
                # For symmetric twin thrusters, lever is half spacing.
                thruster_lever = 0.5 * abs(max(y_vals) - min(y_vals))
            elif len(y_vals) == 1:
                thruster_lever = abs(y_vals[0])

            if len(t_vals) > 0:
                T_act = float(np.mean(t_vals))
        except Exception:
            # Keep defaults if parsing fails.
            pass

    return float(thrust_scale), float(thruster_lever), float(T_act)


def simulation(X0, control, dt, flag=False):
    """
    Simulate the vessel dynamics over time using the Vessel class and parameters from input.yml.
    """
    # Load vessel parameters from input.yml
    base_dir = os.path.dirname(os.path.abspath(__file__))
    input_path = os.path.join(base_dir, 'input.yml')
    params = _safe_load_yaml(input_path)
    thrust_scale, thruster_lever, T_act = _resolve_thruster_model_params(params, base_dir)
#D:\fuel AI prediction in atonomous mode\sookshma\cooperative pursuer1
    # Extract vessel parameters
    vessel_params = {  
        'g':  params['gravity'],
        'rho': params['density'],
        'L': params['length'],
        'U': params['speed'],
        'mass': params['mass'],
        'cog': params['cog'],
        'gyration': params['gyration'],
        'thrust_scale': thrust_scale,
        'thruster_lever': thruster_lever,
         'T_act': T_act,
        'initial_velocity': X0[:6],  # Use the provided initial state for velocity
        'initial_position': X0[6:12],  # Use the provided initial state for position
        'sim_time': params['sim_time'],
        'time_step': dt,  # Override the time step with the provided dt
        'control_type': params['control_type']
    }

    # Load hydrodynamic coefficients from the specified file
    hyd_path = params['hydrodynamic_coefficients']
    if not os.path.isabs(hyd_path):
        hyd_path = os.path.join(base_dir, hyd_path)
    with open(hyd_path, 'r') as hyd_file:
        hydrodynamic_data = yaml.safe_load(hyd_file)

    # control may be CasADi SX/MX (MPC) or numpy (runtime). handle both explicitly.
    if isinstance(control, (ca.SX, ca.MX)):
        if control.numel() == 1:
            act_port = ca.fmin(ca.fmax(control[0], -1.0), 1.0)
            act_stbd = act_port
        else:
            act_port = ca.fmin(ca.fmax(control[0], -1.0), 1.0)
            act_stbd = ca.fmin(ca.fmax(control[1], -1.0), 1.0)
        control_vec = ca.vertcat(act_port, act_stbd)
    else:
        arr = np.asarray(control).astype(float).flatten()
        if arr.size < 2:
            arr = np.array([arr[0], arr[0]], dtype=float)
        control_vec = np.clip(arr[:2], -1.0, 1.0)
    vessel = Vessel(vessel_params, hydrodynamic_data)
    if isinstance(X0, (ca.SX, ca.MX)):
        if X0.numel() >= 14:
            X0_vec = X0[:14]
        else:
            X0_vec = X0[:12]
            X0_vec = ca.vertcat(X0_vec, 0.0, 0.0)
    else:
        X0_arr = np.asarray(X0).flatten()
        if X0_arr.size >= 14:
            X0_vec = X0_arr[:14]
        else:
            X0_vec = np.concatenate([X0_arr[:12], np.zeros(2, dtype=float)])
    vessel.current_state = X0_vec

    # Step simulation with symbolic computation
    new_states = vessel.step(control_vec)

    return new_states
