import numpy as np
from class_navigation import Navigation
from gnc.class_ekf import EKF
from mav_simulator.class_world import World
from rclpy.executors import MultiThreadedExecutor
import rclpy

def calculate_threshold(dt, n_actuators=0):
        # Threshold for correction
        th = np.full(15 + n_actuators, np.inf)

        acc_der_std = 10
        omg_der_std = 10
        act_der_std = 1  # Standard deviation for actuator derivatives

        acc_der_var = acc_der_std**2
        omg_der_var = omg_der_std**2
        act_der_var = act_der_std**2

        # Create a covariance matrix with 6 columns (for angular + linear acceleration)
        cov = np.diag([omg_der_var, omg_der_var, omg_der_var, acc_der_var, acc_der_var, acc_der_var])

        th[9:12] = np.sqrt(omg_der_var/3) * dt
        th[12:15] = np.sqrt(acc_der_var/3) * dt

        # Set thresholds for actuator states
        if n_actuators > 0:
            th[15:15+n_actuators] = np.sqrt(act_der_var) * dt

        th[6:9] = 2.0 * np.linalg.norm(th[12:15]) * dt
        th[3:6] = 2.0 * np.linalg.norm(th[9:12]) * dt

        # Thresholds for positions
        th[0] = 2.0 * np.linalg.norm(th[6:9]) * dt
        th[1] = 2.0 * np.linalg.norm(th[6:9]) * dt
        th[2] = 0.0

        return th, cov

def main():
    world = World('/workspaces/mavlab/inputs/simulation_input.yml')
    vessels = world.vessels
    ekfs = []
    llh0 = world.gps_datum
    for vessel in vessels:
        # Calculate number of actuators (control surfaces + thrusters)
        n_control_surfaces = vessel.n_control_surfaces if hasattr(vessel, 'n_control_surfaces') else 0
        n_thrusters = vessel.n_thrusters if hasattr(vessel, 'n_thrusters') else 0
        n_actuators = n_control_surfaces + n_thrusters
        
        # Update EKF to include actuator states
        _, cov = calculate_threshold(vessel.dt, n_actuators)
        ekfs.append(EKF(
             1/vessel.dt, 
             n_states=15 + n_actuators,  # Add actuator states
             n_inp=1, 
             pro_noise_cov=cov
        ))

    rclpy.init()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    navs = []
    for vessel, ekf in zip(vessels, ekfs):
        # Calculate number of actuators for threshold calculation
        n_control_surfaces = vessel.n_control_surfaces if hasattr(vessel, 'n_control_surfaces') else 0
        n_thrusters = vessel.n_thrusters if hasattr(vessel, 'n_thrusters') else 0
        n_actuators = n_control_surfaces + n_thrusters
        
        th, _ = calculate_threshold(vessel.dt, n_actuators)        
        navs.append(Navigation(vessel, ekf, llh0, th=None))
        executor.add_node(navs[-1])
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for nav in navs:
            nav.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()











