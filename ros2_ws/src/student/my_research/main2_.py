import numpy as np

# import matplotlib
# matplotlib.use('TkAgg') 
import matplotlib
matplotlib.use('TkAgg')
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from evader_guidnce import ONRTBoat
import matplotlib.pyplot as plt
import math
import numpy as np 
import matplotlib.pyplot as plt
#rom nmpc_2 import controller
from kcs_ode import simulation
from distance_plot import plot_all_distances,plot_gradients_over_time


from propeller_control import generate_evasion_propeller
from rudder_control import  generate_evasion_rudder
from visulization import PursuitAnimation
from plot_analysis import generate_analysis_plots, plot_ship_analysis,plot_group_occupied_angle, plot_pursuer_evader_distances, plot_beta_coefficients
import os
from cooperative_pursuer import ApolloniusTradeoffController,compute_local_polar,compute_group_occupied_angle,compute_coverage_angles,compute_coverage_angles,sort_pursuers_by_angle

from tqdm import tqdm
 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray,Float32
def quaternion_to_euler(self,quat):
        """
        Converts a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

        Args:
            x (float): Quaternion x component.
            y (float): Quaternion y component.
            z (float): Quaternion z component.
            w (float): Quaternion w component.

        Returns:
            tuple: (roll, pitch, yaw) in radians.
        """
        x, y, z, w = quat
        # Roll (x-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # Clamp to avoid numerical errors
        pitch = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw 


class PursuerPublisher(Node):
    
    def __init__(self):
        super().__init__('pursuer_publisher')
        self.p1_pub = self.create_publisher(Pose2D, 'pursuer1/pose', 10)
        self.p2_pub = self.create_publisher(Pose2D, 'pursuer2/pose', 10)
        self.p3_pub = self.create_publisher(Pose2D, 'pursuer3/pose', 10)
        self.p1_vel_pub = self.create_publisher(Float32MultiArray, 'pursuer1/velocity', 10)
        self.p2_vel_pub = self.create_publisher(Float32MultiArray, 'pursuer2/velocity', 10)
        self.p3_vel_pub = self.create_publisher(Float32MultiArray, 'pursuer3/velocity', 10)
        self.create_subscription(
            Odometry, 
            '/er/odometry', 
            self.evader_odom_callback, 
            10)
        self.create_subscription(
            Actuator,
            '/sookshma_00/actuator_cmd',
            self.evader_actuator_callback,
            10)
        self.create_subscription(
            Float32,'/kf/yaw',self.evader_yaw_callback,10)
        
        # Initialize evader state storage
        self.evader_state = np.zeros(8)  # [u,v,r,x,y,yaw,rudder,prop]
        self.evader_state_updated = False
        self.yaw=0

        self.evader_guidnce = ONRTBoat()
    def evader_yaw_callback(self, msg):
        """Update evader yaw from yaw message"""
        # Assuming msg is a Float32 with the yaw value in radians
        self.evader_state[5] = msg.data # Update yaw in radians
          # Store yaw in state
        
        self.evader_state_updated = True  # Mark evader
        self.get_logger().info(f"Yaw updated from /kf/psi: {math.degrees(msg.data)}")
    def evader_actuator_callback(self, msg):
        """Update evader control inputs from actuator message"""
        # Assuming actuator message contains rudder and propeller values
        self.evader_state[6] = msg.actuator_values[0]  # rudder
        self.evader_state[7] = msg.actuator_values[1]  # propeller
        self.evader_state_updated = True 
    def evader_odom_callback(self, msg):
        """Update evader state from odometry message"""
        # Extract linear velocities (u,v)
        self.evader_state[0] = msg.twist.twist.linear.x  # surge (u)
        self.evader_state[1] = msg.twist.twist.linear.y  # sway (v)
        
        # Extract angular velocity (r)
        self.evader_state[2] = msg.twist.twist.angular.z  # yaw rate (r)
        
        # Extract position (x,y)
        self.evader_state[3] = msg.pose.pose.position.x
        self.evader_state[4] = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        # quat = [msg.pose.pose.orientation.x,
        #         msg.pose.pose.orientation.y,
        #         msg.pose.pose.orientation.z,
        #         msg.pose.pose.orientation.w]
        # _, _, yaw = quaternion_to_euler(self,quat)
        # yaw_=yaw*57.3
        
        
        self.evader_state_updated = True  
    def publish_state(self, p1, p2, p3):
        def publish_pursuer(pursuer, pose_pub, vel_pub):
            pose_msg = Pose2D()
            pose_msg.x = float(pursuer[3])
            pose_msg.y = float(pursuer[4])
            pose_msg.theta = float(pursuer[5])
            pose_pub.publish(pose_msg)

            vel_msg = Float32MultiArray()
            vel_msg.data = [float(pursuer[0]), float(pursuer[1])]  # u, v
            vel_pub.publish(vel_msg)

        publish_pursuer(p1, self.p1_pub, self.p1_vel_pub)
        publish_pursuer(p2, self.p2_pub, self.p2_vel_pub)
        publish_pursuer(p3, self.p3_pub, self.p3_vel_pub)
    
        
    def get_current_evader_state(self):
        """Get the latest evader state"""
        return self.evader_state.copy()

def run_simulation_with_live_vis(node):
   
    L=2.3
    time_step = 0.5
    simulation_time =150
    n_steps = int(simulation_time / time_step) + 1
    time = np.linspace(0, simulation_time, n_steps) 
   
    X0_pursuer1 = np.array([0.08, .001, .01, 0,27,.06, .001, 2])
    X0_pursuer2 = np.array([0.08, .001, .01, 1, 17, .02, .001,2])
    X0_pursuer3 = np.array([0.08, .001, .01,16, 7,  0.1, .001, 2])
    X0_evader    = np.array([1.3, .001, .01, 12,26, .01, .01, 8])

   
    obst_r = [0.001,0.001]
    obs_pos = ([1,3], [3,2]) 
    goal_evader = np.array([4, 8])   
    

    
  
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
    
   
    fig = plt.figure(figsize=(30, 20))
   
   
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
    
    cooperative_controller = ApolloniusTradeoffController(desired_capture_distance=300.0)
   

    for t in tqdm(time[:-1], desc="Simulation Progress"):
        
        Xp1 = states_pursuer1[-1]
        Xp2 = states_pursuer2[-1]
        Xp3 = states_pursuer3[-1]
        rclpy.spin_once(node, timeout_sec=0.1)  # Process callbacks
        Xe = node.get_current_evader_state()
        print(f"Evader state: u={Xe[0]:.3f}, v={Xe[1]:.3f}, r={Xe[2]:.3f}, x={Xe[3]:.3f}, y={Xe[4]:.3f}, yaw={57.3*Xe[5]:.3f}, rudder={Xe[6]:.3f}, prop={Xe[7]:.3f}")
        # Check if we have valid evader state
        if not node.evader_state_updated:
            print("Waiting for evader state update...")
            continue
        node.publish_state(Xp1, Xp2, Xp3)  # Publish current states to ROS topics
        # nmpc_e ,cost_e= evader_controller.nlpsolve_with_cost(Xe, Xp1, Xp2, Xp3, goal_evader, time, obs_pos,return_cost=True)
        # #ref_pursuer = np.tile(np.array([Xe[0],Xe[3], Xe[4], Xe[5], Xe[7]]), (NP, 1))
        # ...existing code inside the for t in tqdm(time[:-1], ...) loop...
        Xe[0] = 1.2 # Set Xe[0] to 0.5 as per your requirement
        Xe[1] = 0.001
         # Set Xe[1] to 0.001 as per your requirement
        # Print velocities for pursuers and evader
        print(f"Step {t:.2f}s:")
        print(f"  Pursuer1 velocity: u={Xp1[0]:.3f}, v={Xp1[1]:.3f}, speed={np.hypot(Xp1[0], Xp1[1]):.3f}")
        print(f"  Pursuer2 velocity: u={Xp2[0]:.3f}, v={Xp2[1]:.3f}, speed={np.hypot(Xp2[0], Xp2[1]):.3f}")
        print(f"  Pursuer3 velocity: u={Xp3[0]:.3f}, v={Xp3[1]:.3f}, speed={np.hypot(Xp3[0], Xp3[1]):.3f}")
        print(f"  Evader   velocity: u={Xe[0]:.3f}, v={Xe[1]:.3f}, speed={np.hypot(Xe[0], Xe[1]):.3f}")
        # ...existing code...
        evasion_rudder_e= node.evader_guidnce.calculate_rudder()
        prop_command_e = (generate_evasion_propeller(Xe, [Xp1, Xp2, Xp3], goal_evader))
       
        pursuer_states = [Xp1, Xp2, Xp3]
        V_list = [0.5]*8
        theta_1=2*np.arcsin(Xp1[0]/Xe[0])
        theta_2=2*np.arcsin(Xp2[0]/Xe[0])
        theta_3=2*np.arcsin(Xp3[0]/Xe[0])  
        theta_vals = [theta_1, theta_2, theta_3]
        sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, Xe)
        epsilons = compute_coverage_angles(sorted_polar, theta_vals)
        theta_G = compute_group_occupied_angle(theta_vals, epsilons)
        theta_G_history.append(theta_G)
        theta_vals_history.append(theta_vals.copy())
        d_c = 2.3       # Capture radius
        R_c=d_c
        lambda_min = 0.8  # Minimum eigenvalue of the formation matrix
        R_p = 4.0       # Radius of the circumcircle
        R_f=R_p
        R_o = 3.0    # Minimum safe distance between pursuers
        R_b = 2.3
        cooperative_commands = cooperative_controller.compute_tradeoff_command(
            pursuer_states, Xe, V_list, theta_vals,d_c,lambda_min, R_p, R_o, R_b
        )
            # Maximum distance for inter-collision avoidance 

        rclpy.spin_once(node, timeout_sec=0) 
        def ssa(ang, deg=False):
            """
            Smallest Signed Angle (SSA) function to wrap angle to [-180, 180] degrees or 
            [-pi, pi] radians
            
            Args:
                ang (float): Angle to be wrapped
                deg (bool): Return angle in degrees (default: False)
                
            Returns:
                float: Wrapped angle
            """
            if deg:
                ang = (ang + 180) % 360 - 180
            else:
                ang = (ang + np.pi) % (2 * np.pi) - np.pi
            
            return ang 
        def velocity_to_controls(velocity, pursuer_state, evader_state):
            
            vx, vy = velocity
            
            # Get current pursuer position and evader position
            pursuer_pos = pursuer_state[3:5]
            evader_pos = Xe[3:5]
            
            # Calculate relative position vector (from pursuer to evader)
            dx = evader_pos[0] - pursuer_pos[0]
            dy = evader_pos[1] - pursuer_pos[1]
            
            # Calculate alpha (angle to evader)
            _, alpha = compute_local_polar(pursuer_pos,evader_pos)
            
            # Calculate the hunting and surrounding components
            # Hunting direction is towards evader (-alpha)
            # Surrounding direction is perpendicular to hunting (alpha ± π/2)
            
            print("\nVelocity Command Analysis:")
            print(f"Raw velocity command: vx={vx:.2f}, vy={vy:.2f}")
            
           
            
            speed = np.sqrt(vx**2 + vy**2)
            desired_heading = np.arctan2(vy, vx)  # Direction from v_total
            current_heading = ssa(pursuer_state[5])
            
            # Calculate heading error directly from v_total direction
            heading_error = ssa(desired_heading - current_heading)
            
            print("\nHeading Analysis:")
            print(f"Alpha (to evader): {np.degrees(alpha):.1f}°")
            print(f"Current heading: {np.degrees(current_heading):.1f}°")
            print(f"Desired heading from v_total: {np.degrees(desired_heading):.1f}°")
            print(f"Heading error: {np.degrees(heading_error):.1f}°")
            
            # RPM Control based on total velocity magnitude
            nominal_speed = 2
            base_rpm = 2
            rpm = base_rpm * (speed / nominal_speed)
            rpm = np.clip(rpm,1, 2)
            
            
            # Rudder Control with PD controller
            Kp = 10.0
            Kd = 0.8
            yaw_rate = pursuer_state[2]
            rudder_command = Kp * heading_error - Kd * yaw_rate
            
            # Limit rudder 
            max_rudder = 35
            rudder_command = np.clip(rudder_command, -max_rudder, max_rudder)
            
            print("\nControl Output:") 
            print(f"Speed command: {speed:.2f} m/s")
            print(f"RPM command: {rpm:.1f}")
            print(f"Rudder angle: {rudder_command:.1f}°")
            
            return rpm, rudder_command
            
          
        control_p1 = (np.array(velocity_to_controls(cooperative_commands[0], Xp1, Xe))).reshape(1,2)
        control_p2 = (np.array(velocity_to_controls(cooperative_commands[1], Xp2, Xe))).reshape(1,2)
        control_p3 = (np.array(velocity_to_controls(cooperative_commands[2], Xp3, Xe))).reshape(1,2)
       
        print("rudder command",evasion_rudder_e,"propeller command",prop_command_e)

        control_e = (np.array([prop_command_e,evasion_rudder_e ])).reshape(1,2)
       
        # Debug: Log control commands
        print("Control commands:")
        print(f"P1 - RPM: {control_p1[0,0]:.1f}, Rudder: {control_p1[0,1]:.1f}°")
        print(f"P2 - RPM: {control_p2[0,0]:.1f}, Rudder: {control_p2[0,1]:.1f}°")
        print(f"P3 - RPM: {control_p3[0,0]:.1f}, Rudder: {control_p3[0,1]:.1f}°")
        print(f"E  - RPM: {control_e[0,0]:.1f}, Rudder: {control_e[0,1]:.1f}°")
        

      
        control_p1 = np.clip(control_p1, [0.1, -0.61], [2, 0.61])
        control_p2 = np.clip(control_p2, [0.1, -0.61], [2,0.61])
        control_p3 = np.clip(control_p3, [0.1, -0.61], [2, 0.61])
        control_e = np.clip(control_e, [8, -0.61], [9, 0.61])
        
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
        #new_state_evader = simulation(Xe, control_e, time_step, flag=False)
       
        # Calculate distance to goal
        distance_to_goal = np.sqrt((Xe[3] - goal_evader[0])**2 + (Xe[4] - goal_evader[1])**2)
        print(f"Evader - Distance to Goal: {distance_to_goal:.2f}, Cost: ")
        print(f"Evader - Position: x={Xe[3]:.2f}, y={Xe[4]:.2f}, yaw: {np.degrees(Xe[5]):.2f}°")
        # Store new states
        states_pursuer1.append(new_state_pursuer1)
        states_pursuer2.append(new_state_pursuer2)
        states_pursuer3.append(new_state_pursuer3)
        states_evader.append(Xe)  
        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t)
        
        
        plt.pause(0.1)
    # Convert states to numpy arrays
    states_pursuer1 = np.array(states_pursuer1)
    states_pursuer2 = np.array(states_pursuer2)
    states_pursuer3 = np.array(states_pursuer3)
    states_evader = np.array(states_evader)
    # Add to your simulation initialization
    

    # After simulation ends, create the plot
    plot_group_occupied_angle(
        time[:len(theta_G_history)],
        theta_G_history,
        theta_vals_history,
        plots_dir
    )
    from distance_plot import plot_all_distances,plot_gradients_over_time

    # Create distance analysis plots
    plot_all_distances(
        time=time,
        states_pursuers=[states_pursuer1, states_pursuer2, states_pursuer3] ,
        R_o=R_o,
        R_b=R_b, 
        R_c=R_c,
        R_f=R_f,
        plots_dir=plots_dir
    )
    # Add after your existing plot_all_distances call
    plot_gradients_over_time(time=time[:-1],
        
        states_pursuers=[states_pursuer1, states_pursuer2, states_pursuer3],
        R_o=R_o,
        R_b=R_b,
        R_c=R_c,
        R_f=R_f,
        plots_dir=plots_dir
    )
    
    beta_values = [np.array(cooperative_controller.beta_history[i]) for i in range(3)]

# Plot beta coefficients
    beta_time = time[:-1]  # Remove last time step since beta is computed n-1 times
    print(f"Time array length: {len(beta_time)}, Beta values length: {len(beta_values[0])}")

    # Plot beta coefficients
    from plot_analysis import plot_beta_coefficients
    plot_beta_coefficients(
        time=beta_time,
        beta_values_per_pursuer=beta_values,
        plots_dir=plots_dir
    )
    plt.ioff()
               
    plt.close('all')
    
    plt.ioff()
               
    plt.close('all')
    # Optionally, after simulation you can still save the complete animation:
   

    return (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
            time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2, X0_pursuer3, X0_evader,
            commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e,
            commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e)

   
    
   

if __name__ == "__main__":
    rclpy.init()
    node = PursuerPublisher()
    
    #print("Starting simulation...")  
    plots_dir="cooperative_34"
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)
        print(f"created directory : {plots_dir}")
    print("starting simulation")
    simulation_results = run_simulation_with_live_vis(node)
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
    
   
    generate_analysis_plots(
    states_pursuers=[states_pursuer1, states_pursuer2, states_pursuer3],
    states_evader=states_evader,
    commanded_rudders_pursuers=[commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3],
    commanded_rudders_evader=commanded_rudders_e,
    commanded_props_pursuers=[commanded_props_p1, commanded_props_p2, commanded_props_p3],
    commanded_props_evader=commanded_props_e,
    plots_dir=plots_dir
)
    
    
    # Plot pursuer-evader distances
    plot_pursuer_evader_distances(
        time=time,
        states_pursuers=[states_pursuer1, states_pursuer2, states_pursuer3],
        states_evader=states_evader,
        capture_radius=2.3,  # Same as d_c
        plots_dir=plots_dir
    )
    print("Analysis plots generated and saved.")
      
    plt.close('all')
    