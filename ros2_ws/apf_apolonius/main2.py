import numpy as np

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

import numpy as np
import casadi as cd 
import matplotlib.pyplot as plt
from mpc_evader import controller
from kcs_ode import simulation
from module_kinematics import ssa

from visulization import PursuitAnimation 
from plot_analysis import generate_analysis_plots
import os
from cooperative_pursuer import ApolloniusTradeoffController,compute_local_polar,compute_group_occupied_angle,compute_coverage_angles, sort_pursuers_by_angle

from tqdm import tqdm
 



def run_simulation_with_live_vis():
   
    NP = 10
    NC = 4
    Q = np.array([10, 10, 1])
    
    time_step = 0.5
    simulation_time =6
    n_steps = int(simulation_time / time_step) + 1
    time = np.linspace(0, simulation_time, n_steps) 
    evader_x, evader_y = 50, 50  # Change as needed
  

    # 2. Define triangle (side length L)
    L = 15 # Side length of triangle

    # 3. Calculate pursuer positions (equilateral triangle around centroid)
    angle_offset = np.deg2rad(120)
    p1_x = evader_x + L * np.cos(angle_offset)
    p1_y = evader_y + L * np.sin(angle_offset)

    p2_x = evader_x + L * np.cos(angle_offset + 2 * np.pi / 3)
    p2_y = evader_y + L * np.sin(angle_offset + 2 * np.pi / 3)

    p3_x = evader_x + L * np.cos(angle_offset + 4 * np.pi / 3)  
    p3_y = evader_y + L * np.sin(angle_offset + 4 * np.pi / 3)
   
    X0_pursuer1 = np.array([0.4, 0.01, 0.01, 0.01, 0.01, 0.01, p1_x, p1_y, 0, 0.0, 0.0, .2, 0.1])
    X0_pursuer2 = np.array([0.4, 0.01, 0.01, 0.01, 0.01, 0.01, p2_x, p2_y, 0.0, 0.0, 0.0, .1, 0.1])
    X0_pursuer3 = np.array([0.4, 0.01, 0.01, 0.01, 0.01, 0.01, p3_x, p3_y, 0.0, 0.0, 0.0, -3.14, 0.1])
    X0_evader    = np.array([0.5, 0.01, 0.01, 0.01, 0.01, 0.01, evader_x,evader_y, 0.0, 0.0, 0.0, .1,0.1])

   
    obst_r = [0.1,0.1]
    obs_pos = ([20,20], [30,40]) 
    goal_evader = np.array([100, 100])   
    

    
  
    states_pursuer1 = [X0_pursuer1]
    states_pursuer2 = [X0_pursuer2]
    states_pursuer3 = [X0_pursuer3]
    states_evader    = [X0_evader]
   

    
 
    print("Initial states:")
    print("Pursuer1:", X0_pursuer1)
    print("Pursuer2:", X0_pursuer2)
    print("Pursuer3:", X0_pursuer3)
    print("Evader:", X0_evader)

   
    commanded_rudders_p1 = []
    commanded_rudders_p2 = []
    commanded_rudders_p3 = []
    commanded_rudders_e = []
    commanded_props_p1 = []
    commanded_props_p2 = []
    commanded_props_p3 = []
    commanded_props_e = []
    theta_G_history = []
    theta_vals_history = []
   
    fig = plt.figure(figsize=(20, 10))
    evader_controller=controller(time_step,NP,NC,Q,obst_r)
   
    live_anim = PursuitAnimation(
        states_pursuer1 = np.array(states_pursuer1),
        states_pursuer2 = np.array(states_pursuer2),
        states_pursuer3 = np.array(states_pursuer3),
        states_evader    = np.array(states_evader),
        time = time,
        obs_pos = obs_pos,
        obst_r = obst_r,
        goal_evader = goal_evader,
        X0_pursuer1 = X0_pursuer1,
        X0_pursuer2 = X0_pursuer2,
        X0_pursuer3 = X0_pursuer3,
        X0_evader = X0_evader,
        
    )
    
    cooperative_controller = ApolloniusTradeoffController(desired_capture_distance=5.0)
    all_predicted_paths = [] 
    for t in tqdm(time[:-1], desc="Simulation Progress"):
        
        Xp1 = states_pursuer1[-1]
        Xp2 = states_pursuer2[-1]
        Xp3 = states_pursuer3[-1]
        Xe  = states_evader[-1]
       
       
       
        control_e = np.zeros((NP, 2))
       
        pursuer_states = [Xp1, Xp2, Xp3]
        V_list = [.5, 0.5,0.5] 
        Xe[0]=0.55 
        theta_1=2*np.arcsin(Xp1[0]/Xe[0])
        theta_2=2*np.arcsin(Xp2[0]/Xe[0])
        theta_3=2*np.arcsin(Xp3[0]/Xe[0])  
        theta_vals = [theta_1, theta_2, theta_3]

        sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, Xe)
        epsilons = compute_coverage_angles(sorted_polar, theta_vals)
        theta_G = compute_group_occupied_angle(theta_vals, epsilons)
        theta_G_history.append(theta_G)
        theta_vals_history.append(theta_vals.copy())
        d_c = 2.3      # Capture radius
        R_c=d_c
        lambda_min = 0.8  # Minimum eigenvalue of the formation matrix
        R_p = 3.0       # Radius of the circumcircle
        R_f=R_p
        R_o = 2.0   # Minimum safe distance between pursuers
        R_b = 2.3
        cooperative_commands = cooperative_controller.compute_tradeoff_command(
            pursuer_states, Xe, V_list, theta_vals,d_c,lambda_min, R_p, R_o, R_b
        )
        
        from control_utils import velocity_to_controls

# Replace the inline definition of velocity_to_controls with the imported function
        control_p1 = (np.array(velocity_to_controls(cooperative_commands[0], Xp1, Xe))).reshape(1, 2)
        control_p2 = (np.array(velocity_to_controls(cooperative_commands[1], Xp2, Xe))).reshape(1, 2)
        control_p3 = (np.array(velocity_to_controls(cooperative_commands[2], Xp3, Xe))).reshape(1, 2)
        
        
        nmpc_e,predicted_path=evader_controller.nlpsolve_with_cost(Xe, Xp1, Xp2, Xp3, goal_evader, time, obs_pos)
        all_predicted_paths.append(predicted_path)
        for i in range(NP):
            control_e[i]=[nmpc_e[i,0], nmpc_e[i,1]]
            print(f"Evader Control - RPM: {control_e[i,0]:.1f}, Rudder at a main : {control_e[i,1]:.1f}°")
       
        # Debug: Log control commands
        print("Control at simulation uy ____________________commands:")
       
        print(f"E _________________ - RPM: {control_e[0,0]:.1f}, Rudder: {control_e[0,1]:.1f}°")
        

      
        control_p1 = np.clip(control_p1, [0, -35], [2, 35])
        control_p2 = np.clip(control_p2, [0, -35], [2,35])
        control_p3 = np.clip(control_p3, [0, -35], [2, 35])
        control_e = np.clip(control_e, [0, -35], [3,35])
        
        # Store commands for analysisf
        commanded_rudders_p1.append(control_p1[0,1])
        commanded_rudders_p2.append(control_p2[0,1])
        commanded_rudders_p3.append(control_p3[0,1])
        commanded_rudders_e.append(control_e[0,1])
        
        commanded_props_p1.append(control_p1[0,0])
        commanded_props_p2.append(control_p2[0,0])
        commanded_props_p3.append(control_p3[0,0])
        commanded_props_e.append(control_e[0,0])
       
        
        
        # Simulate one step
        new_state_pursuer1 = simulation(Xp1, control_p1, time_step, flag=False)
        new_state_pursuer2 = simulation(Xp2, control_p2, time_step, flag=False)
        new_state_pursuer3 = simulation(Xp3, control_p3, time_step, flag=False)
        new_state_evader = simulation(Xe, control_e, time_step, flag=False)
       
        # Calculate distance to goal
        distance_to_goal = np.sqrt((Xe[6] - goal_evader[0])**2 + (Xe[7] - goal_evader[1])**2)
        print(f"Evader - Distance to Goal: {distance_to_goal:.2f}, Cost: ")
        
        # Store new states
        states_pursuer1.append(new_state_pursuer1)
        states_pursuer2.append(new_state_pursuer2)
        states_pursuer3.append(new_state_pursuer3)
        states_evader.append(new_state_evader)  
        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t,predicted_path=predicted_path)
        
        
        plt.pause(0.1)
    # Convert states to numpy arrays
    states_pursuer1 = np.array(states_pursuer1)
    states_pursuer2 = np.array(states_pursuer2)
    states_pursuer3 = np.array(states_pursuer3)
    states_evader = np.array(states_evader)
    
   
    
    
    plt.ioff()
               
    plt.close('all')
    
    plt.ioff()
               
    plt.close('all')
    # Optionally, after simulation you can still save the complete animation:
   

    return (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
            time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2, X0_pursuer3, X0_evader,
            commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e,
            commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e,all_predicted_paths)

   
    
   

if __name__ == "__main__":
    
    
    #print("Starting simulation...")  
    plots_dir="cooperative_21"
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)
        print(f"created directory : {plots_dir}")
    print("starting simulation")
    simulation_results = run_simulation_with_live_vis()
    # Run simulation
    (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
     time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2,
     X0_pursuer3, X0_evader, commanded_rudders_p1, commanded_rudders_p2,
     commanded_rudders_p3, commanded_rudders_e, commanded_props_p1, commanded_props_p2,
     commanded_props_p3, commanded_props_e,all_pedicted_path) = simulation_results
    
    print("Simulation complete. Creating animation...")
    animation_path=os.path.join(plots_dir,'pursuit_animation_formation.gif')
     
    animator = PursuitAnimation(
        states_pursuer1=states_pursuer1,
        states_pursuer2=states_pursuer2,
        states_pursuer3=states_pursuer3,
        states_evader=states_evader,
        time=time,
        obs_pos=obs_pos,
        obst_r=obst_r,
        goal_evader=goal_evader,
        X0_pursuer1=X0_pursuer1,
        X0_pursuer2=X0_pursuer2,
        X0_pursuer3=X0_pursuer3,
        X0_evader=X0_evader,predicted_path=all_pedicted_path
        
    )
    
    print("Saving animation...") 
    animator.create_animation(animation_path)
    print("Animation saved as 'pursuit_animation_formation.gif'") 
    
    print("Generating analysis plots...")
    from plot_analysis import generate_analysis_plots
    
    generate_analysis_plots(states_pursuer1, states_pursuer2, states_pursuer3, 
                           states_evader, commanded_rudders_p1, commanded_rudders_p2,
                           commanded_rudders_p3, commanded_rudders_e,
                           commanded_props_p1, commanded_props_p2,
                           commanded_props_p3, commanded_props_e,plots_dir=plots_dir)
    print("Analysis plots generated and saved.") 
    plt.close('all')
    