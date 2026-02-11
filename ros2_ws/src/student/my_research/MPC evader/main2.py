import numpy as np

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt

import numpy as np
import casadi as cd 
import matplotlib.pyplot as plt
from nmpc_2 import controller
from kcs_ode import simulation
from formation_controller import FormationController

from visulization import PursuitAnimation
from plot_analysis import generate_analysis_plots, plot_ship_analysis
import os

from tqdm import tqdm
 



def run_simulation_with_live_vis():
    # Define parameters (as per your simulation)
    NP = 10
    NC = 4
    Q = np.array([1, 1, 1])
    
    time_step = 0.5# 100ms time step
    simulation_time = 100  # 30 seconds total
    n_steps = int(simulation_time / time_step) + 1
    time = np.linspace(0, simulation_time, n_steps)
    # Initial states for pursuers and evader
    X0_pursuer1 = np.array([.5, .1, .1, 300, 500, .01, .01, 10])
    X0_pursuer2 = np.array([.5, .1, .1, 400, 800, .01, .01, 10])
    X0_pursuer3 = np.array([.5, .1, .1, 500, 600, .01, .01, 10])
    X0_evader    = np.array([.5, .1, .1, 400,   1,  .1, .01, 10])

    # Obstacles positions and evader goal
    obst_r = [40,30]
    obs_pos = ([400,100], [400,500])
    goal_evader = np.array([800, 800])
    pursuer_controller1 = controller(time_step, NP, NC, Q, obst_r)
    pursuer_controller2 = controller(time_step, NP, NC, Q, obst_r)
    pursuer_controller3 = controller(time_step, NP, NC, Q, obst_r)
    evader_controller = controller(time_step, NP, NC, Q, obst_r)

    
    # Create a time vector for simulation
    
    # Initialize state lists
    states_pursuer1 = [X0_pursuer1]
    states_pursuer2 = [X0_pursuer2]
    states_pursuer3 = [X0_pursuer3]
    states_evader    = [X0_evader]
    h = time[1] - time[0] 

    # Reference goals

    goal_evader = np.array([800,800])
 
    print("Initial states:")
    print("Pursuer1:", X0_pursuer1)
    print("Pursuer2:", X0_pursuer2)
    print("Pursuer3:", X0_pursuer3)
    print("Evader:", X0_evader)

    # Initialize lists to store commanded rudders and propellers
    commanded_rudders_p1 = []
    commanded_rudders_p2 = []
    commanded_rudders_p3 = []
    commanded_rudders_e = []
    commanded_props_p1 = []
    commanded_props_p2 = []
    commanded_props_p3 = []
    commanded_props_e = []
    
    # Simulation loop    plt.ion()
    fig = plt.figure(figsize=(30, 15))
   
    # Initialize live visualization using the PursuitAnimation class.
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
    

     # Turn on interactive mode

    # Simulation loop with live visualization
    for t in tqdm(time[:-1], desc="Simulation Progress"):
        # 1. Get current states
        
        # Retrieve latest states
        Xp1 = states_pursuer1[-1]
        Xp2 = states_pursuer2[-1]
        Xp3 = states_pursuer3[-1]
        Xe  = states_evader[-1]
        print("Xe shape",Xe.shape)
        
        # --- Compute control commands and update states ---
        # Here you would use your NMPC, formation, pursuit/evasion controls etc.
        # For demonstration, we use the dummy control.
        ref_pursuer = np.tile(np.array([Xe[1],Xe[3], Xe[4], Xe[5], Xe[7]]), (NP, 1))
        
        # 2. Get NMPC solutions
        nmpc_p1,cost_p1 = pursuer_controller1.nlpsolve(ref_pursuer, Xp1, time, obs_pos, pursuer_states=[Xp2, Xp3],return_cost=True)
        nmpc_p2,cost_p2 = pursuer_controller2.nlpsolve(ref_pursuer, Xp2, time, obs_pos, pursuer_states=[Xp1, Xp3],return_cost=True)
        nmpc_p3,cost_p3 = pursuer_controller3.nlpsolve(ref_pursuer, Xp3, time, obs_pos, pursuer_states=[Xp1, Xp2],return_cost=True)
        nmpc_e ,cost_e= evader_controller.nlpsolve_with_cost(Xe, Xp1, Xp2, Xp3, goal_evader, time, obs_pos,return_cost=True)
        
        # 3. Calculate formation positions and assignments
        
        # 6. Initialize control arrays
        control_p1 = np.zeros((NP, 2))
        control_p2 = np.zeros((NP, 2))
        control_p3 = np.zeros((NP, 2))
        control_e = np.zeros((NP,2))
        
        for i in range(NP):
           
       
         control_p1[i] = [
                nmpc_p1[i,0],  # propeller
                nmpc_p1[i,1]   # rudder
            ]
         
            
        
        # Pursuer 2
         
         control_p2[i] = [
                nmpc_p2[i,0],  # propeller
                nmpc_p2[i,1]   # rudder
            ]
         
         
         control_p3[i] = [
                nmpc_p3[i,0],  
                nmpc_p3[i,1]  
            ]
         
            
         
         control_e[i] = [nmpc_e[i,0], nmpc_e[i,1]]
        print("ruddercommand value",control_p1[0,1])
        # Clip controls to physical limits
        control_p1 = np.clip(control_p1, [0, -35], [30, 35])
        control_p2 = np.clip(control_p2, [0, -35], [30, 35])
        control_p3 = np.clip(control_p3, [0, -35], [30, 35])
        control_e = np.clip(control_e, [0, -35], [20, 35])
        
        # Store commands for analysisf
        commanded_rudders_p1.append(control_p1[0,1])
        commanded_rudders_p2.append(control_p2[0,1])
        commanded_rudders_p3.append(control_p3[0,1])
        commanded_rudders_e.append(control_e[0,1])
        
        commanded_props_p1.append(control_p1[0,0])
        commanded_props_p2.append(control_p2[0,0])
        commanded_props_p3.append(control_p3[0,0])
        commanded_props_e.append(control_e[0,0])
        # Debug prints
        print(f"\nTime step: {t:.1f}")
        print("Control commands:")
        print(f"P1 - RPM: {control_p1[0,0]:.1f}, Rudder: {control_p1[0,1]:.1f}°")
        print(f"P2 - RPM: {control_p2[0,0]:.1f}, Rudder: {control_p2[0,1]:.1f}°")
        print(f"P3 - RPM: {control_p3[0,0]:.1f}, Rudder: {control_p3[0,1]:.1f}°")
        print(f"E  - RPM: {control_e[0,0]:.1f}, Rudder: {control_e[0,1]:.1f}°")
        
        # Simulate one step
        new_state_pursuer1 = simulation(Xp1, control_p1, time_step, flag=True)
        new_state_pursuer2 = simulation(Xp2, control_p2, time_step, flag=True)
        new_state_pursuer3 = simulation(Xp3, control_p3, time_step, flag=True)
        new_state_evader = simulation(Xe, control_e, time_step, flag=True)
        
        # Store new states
        states_pursuer1.append(new_state_pursuer1[:, -1])
        states_pursuer2.append(new_state_pursuer2[:, -1])
        states_pursuer3.append(new_state_pursuer3[:, -1])
        states_evader.append(new_state_evader[:, -1])
        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t,cost=(cost_p1,cost_p2,cost_p3,cost_e)
        )
        
        plt.pause(0.1)
    # Convert states to numpy arrays
    states_pursuer1 = np.array(states_pursuer1)
    states_pursuer2 = np.array(states_pursuer2)
    states_pursuer3 = np.array(states_pursuer3)
    states_evader = np.array(states_evader)
    
   
    
    
    plt.ioff()
               # Turn off interactive mode
      # Keep the final frame displayed
    plt.close('all')
    
    plt.ioff()
               # Turn off interactive mode
      # Keep the final frame displayed
    plt.close('all')
    # Optionally, after simulation you can still save the complete animation:
   

    return (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
            time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2, X0_pursuer3, X0_evader,
            commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e,
            commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e)

   
    
   

if __name__ == "__main__":
    
    
    #print("Starting simulation...")  
    plots_dir="plots33"
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
     commanded_props_p3, commanded_props_e) = simulation_results
    
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
        X0_evader=X0_evader,
        
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
    