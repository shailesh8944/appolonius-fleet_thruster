import casadi as ca

import yaml
from casadi_class_vessel import Vessel
import numpy as np

def simulation(X0, control, dt, flag=False):
    """
    Simulate the vessel dynamics over time using the Vessel class and parameters from input.yml.
    """
    # Load vessel parameters from input.yml
    with open('input.yml', 'r') as file:
        params = yaml.safe_load(file)
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
        'thrust_scale': params.get('thrust_scale', 5.0),
        'thruster_lever': params.get('thruster_lever', 0.21),
        'T_act': params.get('T_act', 0.2),
        'initial_velocity': X0[:6],  # Use the provided initial state for velocity
        'initial_position': X0[6:12],  # Use the provided initial state for position
        'sim_time': params['sim_time'],
        'time_step': dt,  # Override the time step with the provided dt
        'control_type': params['control_type']
    }

    # Load hydrodynamic coefficients from the specified file
    with open(params['hydrodynamic_coefficients'], 'r') as hyd_file:
        hydrodynamic_data = yaml.safe_load(hyd_file)

    # Initialize the Vessel instanc
      # control may be CasADi SX/MX (MPC) or numpy (runtime). handle both explicitly.
    if isinstance(control, (ca.SX, ca.MX)):
        act_port = ca.fmin(ca.fmax(control[0], -1.0), 1.0)
        act_stbd = ca.fmin(ca.fmax(control[1], -1.0), 1.0)
        control_vec = ca.vertcat(act_port, act_stbd)
    else:
        arr = np.asarray(control).astype(float).flatten()
        if arr.size < 2:
            raise ValueError("Thruster control must have 2 elements: [act_port, act_stbd].")
        control_vec = np.clip(arr[:2], -1.0, 1.0)
    vessel = Vessel(vessel_params, hydrodynamic_data) 
    x0 = np.asarray(X0).flatten()
    if x0.size >= 14:
        vessel.current_state = x0[:14]
    else:
        vessel.current_state = np.concatenate([x0[:12], np.zeros(2, dtype=float)])

    # Step simulation with symbolic computation
    new_states = vessel.step(control_vec)

    return new_states
