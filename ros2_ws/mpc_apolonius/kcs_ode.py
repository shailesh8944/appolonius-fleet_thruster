
import yaml
from class_vessel import Vessel
import numpy as np

def simulation(X0, control, dt, flag=False):
    """
    Simulate the vessel dynamics over time using the Vessel class and parameters from input.yml.
    """
    # Load vessel parameters from input.yml
    with open('input.yml', 'r') as file:
        params = yaml.safe_load(file)
#workspaces/mavlab/ros2_ws/src/mpc_apolonius
    # Extract vessel parameters
    vessel_params = {
        'g': params['gravity'],
        'rho': params['density'],
        'L': params['length'],
        'U': params['speed'],
        'mass': params['mass'],
        'cog': params['cog'],
        'gyration': params['gyration'],
        'T_delta': params['T_delta'],
        'initial_velocity': X0[:6],  # Use the provided initial state for velocity
        'initial_position': X0[6:12],  # Use the provided initial state for position
        'sim_time': params['sim_time'],
        'time_step': dt,  # Override the time step with the provided dt
        'control_type': params['control_type']
    }

    # Load hydrodynamic coefficients from the specified file
    with open(params['hydrodynamic_coefficients'], 'r') as hyd_file:
        hydrodynamic_data = yaml.safe_load(hyd_file)

    # Initialize the Vessel instance
    vessel = Vessel(vessel_params, hydrodynamic_data)

    # Set the control inputs
    print(f"Control inputs before simulation my rudder angle: {control[0, 1]} ")
    # print(f"vessels speed: {vessel.current_state[0]} ")
    # print(f"vessels heading: {vessel.current_state[5]} ")
    # Convert the rudder angle from degrees to radians
    rudder_angle_radians = np.radians(control[0, 1])
    print("rudder angle before simulation",rudder_angle_radians)  # Convert to radians
    print(f"Rudder command (deg): {control[0, 1]:.2f}")
    print(f"Rudder command (rad): {rudder_angle_radians:.4f}")

    # Set the rudder command in radians
    vessel.delta_c = rudder_angle_radians

    # Step the vessel forward in time
    vessel.step()
    print(f"Control inputs my rudder angle: {control[0, 1]} ")
    print(f"vessels speed: {vessel.current_state[0]} ")
    print(f"vessels heading: {vessel.current_state[5]} ")
    # Return the updated state
    return vessel.current_state