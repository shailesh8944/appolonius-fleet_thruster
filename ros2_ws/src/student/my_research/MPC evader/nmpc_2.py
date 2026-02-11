import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from math import sin,cos,sqrt,pi,exp    
from CASadi_kcs_ode import _mmgder  # Import the MMG3 dynamics
from formation_controller import FormationController
from scipy.integrate import solve_ivp




class controller():  
    def __init__(self, time_step, NP, NC, Q, r):
        self.time_step = time_step
        self.P = NP  # Prediction horizon
        self.NC = NC  # Control horizon
        self.Q = Q   # Cost weights
        self.r = r   # Obstacle radius
        self.current_state = None 
        
        self.formation_controller = FormationController() # Store current state
       
        self.opt_var = cd.vertcat(
        cd.SX.sym('n_prop', self.P),    # Propeller commands
        cd.SX.sym('rudder', self.P)     # Rudder commands in degrees
    )
        self.control_variable = cd.reshape(self.opt_var, (self.P, 2))  # Reshape to matrix form
        
        # Split into propeller and rudder components
        self.n_prop_var = self.control_variable[:, 0]
        self.rudder_var = self.control_variable[:, 1]
        print("\n=== Initialization ===")
        print(f"opt_var shape: {self.opt_var.shape}")
        print(f"control_variable shape: {self.control_variable.shape}")
        
    
    def prediction_model(self, states, n_prop_cmd,rudder_cmd, h):
     """
     Predicts ship's future states using MMG3 dynamics
     
     """
     max_prop_rpm=4
     max_rudder_angle=0.61
     if isinstance(states,np.ndarray):
        if isinstance(n_prop_cmd, (int, float)) and isinstance(rudder_cmd, (int, float)):
            print(f"Debug - prediction_model input:prop={n_prop_cmd},rudder={rudder_cmd}")
        else:
            print("Debug - prediction_model with symbolic control inputs")
   
    
     rudder_rad = cd.fmin(cd.fmax(rudder_cmd, -max_rudder_angle), max_rudder_angle)
        # Create a CasADi vector for control
     control = cd.vertcat(n_prop_cmd, rudder_rad)  
      
    
     
  
     
     f=_mmgder(states,control)
     k1=f
     k2=_mmgder(states+h/2*k1,control)
     k3=_mmgder(states+h/2*k2,control)
     k4=_mmgder(states+h*k3,control)
     xinit1=states+(h/6)*(k1+2*k2+2*k3+k4) 
     
    # Normalize and clip states using CasADi functions
     #xinit1[5] = cd.arctan2(xinit1[1], xinit1[0])
     xinit1[6] = cd.fmin(cd.fmax(xinit1[6], -max_rudder_angle * cd.pi/180), 
                        max_rudder_angle * cd.pi/180)
     xinit1[7] = cd.fmin(cd.fmax(xinit1[7], 0), max_prop_rpm)
     
     return xinit1   
    
   

    def ssa(self, ang, deg=False):
        """
        Smallest Signed Angle function adapted for CasADi framework
        Args:
            ang: angle (CasADi symbolic or numerical)
            deg: boolean flag for degrees (True) or radians (False)
        Returns:
            normalized angle in range [-pi, pi] or [-180, 180]
        """
        # import casadi as cd
        
        if deg:
            # For degrees: normalize to [-180, 180]
            # Using CasADi's fmod and if_else functions
            normalized = cd.fmod(ang + 180, 360)
            return cd.if_else(normalized > 0,
                             normalized - 180,
                             normalized + 180)
        else:
            # For radians: normalize to [-pi, pi]
            # Using CasADi's fmod and if_else functions
            normalized = cd.fmod(ang + cd.pi, 2 * cd.pi)
            return cd.if_else(normalized > 0,
                             normalized - cd.pi,
                             normalized + cd.pi)

    # def ssa(ang, deg=False):
    #         """
    #         Smallest Signed Angle (SSA) function to wrap angle to [-180, 180] degrees or 
    #         [-pi, pi] radians
            
    #         Args:
    #             ang (float): Angle to be wrapped
    #             deg (bool): Return angle in degrees (default: False)
                
    #         Returns:
    #             float: Wrapped angle
    #         """
    #         if deg:
    #             ang = (ang + 180) % 360 - 180
    #         else:
    #             ang = (ang + np.pi) % (2 * np.pi) - np.pi

    def evader_cost(self, Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos):
        #print("\n=== Starting evader cost calculation ===")
        h = t[1] - t[0]
        xinit = Xe.copy()
        prediction = cd.SX(1, 8)
        [cx, cy] = obs_pos
        r = self.r
        self.g = []
        self.lbg = []
        self.ubg = []
        
        # Adjusted weights for exponential costs
        w_goal = 0.1     # Goal weight
        w_escape = 0.003    # Escape weight
        
        w_velocity = 0.02    # Velocity alignment weight 
        
        # Parameters for exponential functions
        k_goal = 0.1     # Goal exponential decay rate
        k_escape = 0.01    # Escape exponential decay rate
        k_vel = 0.05  
        w_heading = 0.01    # Velocity alignment exponential factor
        
        total_cost = 0
        
        # Predict states over the horizon
        for i in range(self.P):
            # Get control input for this timestep
            n_prop = self.control_variable[i, 0]
            rudder = self.control_variable[i, 1]
            
            
            # Update state using prediction model
            newstate = self.prediction_model(xinit, n_prop,rudder, h)
            xinit = newstate
            prediction = cd.vertcat(prediction, xinit.T)
            
            # Obstacle avoidance with exponential cost
            x, y = newstate[3], newstate[4]
            psi= newstate[5]
            u= newstate[0]
            v= newstate[1]
            
           
        
        prediction = prediction[1:, :]
        
        # Extract predicted states
        x_prediction = prediction[:, 3]
        y_prediction = prediction[:, 4]
        psi_prediction = prediction[:, 5]
        u_prediction = prediction[:, 0]
        v_prediction = prediction[:, 1]
        
        # Goal-seeking cost with exponential convergence
        distance_to_goal = cd.sqrt(
            (x_prediction - goal_evader[0])**2 +
            (y_prediction - goal_evader[1])**2
        )
        desired_heading = cd.arctan2(goal_evader[1] - y, goal_evader[0] - x)
        heading_error = self.ssa(desired_heading - psi)
        
        goal_cost =  (
            w_goal*distance_to_goal * cd.exp(-k_goal * distance_to_goal) +  # Distance term
            w_heading * heading_error**2                             # Heading alignment
        )
        
        # Pursuer avoidance with exponential repulsion
        distance_to_p1 = cd.sqrt((x_prediction - Xp1[3])**2 + (y_prediction - Xp1[4])**2)
        distance_to_p2 = cd.sqrt((x_prediction - Xp2[3])**2 + (y_prediction - Xp2[4])**2)
        distance_to_p3 = cd.sqrt((x_prediction - Xp3[3])**2 + (y_prediction - Xp3[4])**2)
        min_safe_distance=23
      
        escape_cost = w_escape * (
            cd.exp(-k_escape * (distance_to_p1 - min_safe_distance)) +
            cd.exp(-k_escape * (distance_to_p2 - min_safe_distance)) +
            cd.exp(-k_escape * (distance_to_p3 - min_safe_distance))
        )
       
        
        # Calculate desired escape direction
        pursuer_center_x = (Xp1[3] + Xp2[3] + Xp3[3]) / 3
        pursuer_center_y = (Xp1[4] + Xp2[4] + Xp3[4]) / 3
                # Calculate individual threat levels from each pursuer
        threat_p1 = cd.exp(-k_escape * distance_to_p1)
        threat_p2 = cd.exp(-k_escape * distance_to_p2)
        threat_p3 = cd.exp(-k_escape * distance_to_p3)
        threat_level = cd.fmax(cd.fmax(threat_p1, threat_p2), threat_p3)
        
        # Enhanced escape direction calculation
        escape_dir_x = x_prediction - pursuer_center_x
        escape_dir_y = y_prediction - pursuer_center_y
        goal_dir_x=goal_evader[0]-x_prediction
        goal_dir_y=goal_evader[1]-y_prediction 
        # Normalize directions
        goal_norm = cd.sqrt(goal_dir_x**2 + goal_dir_y**2) + 1e-6
        escape_norm = cd.sqrt(escape_dir_x**2 + escape_dir_y**2) + 1e-6
        
        goal_dir_x = goal_dir_x / goal_norm
        goal_dir_y = goal_dir_y / goal_norm
        escape_dir_x = escape_dir_x / escape_norm
        escape_dir_y = escape_dir_y / escape_norm
        # Weighted combination of goal direction and escape direction
        goal_dir_x = goal_evader[0] - x_prediction
        goal_dir_y = goal_evader[1] - y_prediction
        escape_dir_x = x_prediction - pursuer_center_x
        escape_dir_y = y_prediction - pursuer_center_y
        
        # Normalize directions
        goal_norm = cd.sqrt(goal_dir_x**2 + goal_dir_y**2) + 1e-6
        escape_norm = cd.sqrt(escape_dir_x**2 + escape_dir_y**2) + 1e-6
        
        goal_dir_x = goal_dir_x / goal_norm
        goal_dir_y = goal_dir_y / goal_norm
        escape_dir_x = escape_dir_x / escape_norm
        escape_dir_y = escape_dir_y / escape_norm
        
        # Combine directions based on distance to pursuers
        min_distance_to_pursuers = cd.fmin(cd.fmin(distance_to_p1, distance_to_p2), distance_to_p3)
        threat_level = cd.exp(-k_escape * min_distance_to_pursuers)
        threat_weight = cd.if_else(threat_level > 0.5, 0.9, threat_level * 1.8)
        desired_dir_x = (1 - threat_weight) * goal_dir_x + threat_weight * escape_dir_x
        desired_dir_y = (1 - threat_weight) * goal_dir_y + threat_weight * escape_dir_y
        def point_in_triangle(px, py, x1, y1, x2, y2, x3, y3):
            # Barycentric technique for CasADi
            denom = (y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3)
            w1 = ((y2 - y3)*(px - x3) + (x3 - x2)*(py - y3)) / (denom + 1e-8)
            w2 = ((y3 - y1)*(px - x3) + (x1 - x3)*(py - y3)) / (denom + 1e-8)
            w3 = 1 - w1 - w2
            # Inside if all weights in [0,1]
            return cd.if_else(
                cd.fmin(cd.fmin(w1, w2), w3) >= 0,
                cd.if_else(cd.fmax(cd.fmax(w1, w2), w3) <= 1, 1, 0),
                0
            )
        def perp_dist(px, py, x1, y1, x2, y2):
            # Perpendicular distance from (px,py) to line (x1,y1)-(x2,y2)
            num = cd.fabs((y2 - y1)*px - (x2 - x1)*py + x2*y1 - y2*x1)
            denom = cd.sqrt((y2 - y1)**2 + (x2 - x1)**2) + 1e-8
            return num / denom
        
        x1, y1 = Xp1[3], Xp1[4]
        x2, y2 = Xp2[3], Xp2[4]
        x3, y3 = Xp3[3], Xp3[4]

        # Use predicted evader position at last step
        px = x_prediction[-1]
        py = y_prediction[-1]

        inside = point_in_triangle(px, py, x1, y1, x2, y2, x3, y3)

        # Perpendicular distances to each side
        a = perp_dist(px, py, x1, y1, x2, y2)
        b = perp_dist(px, py, x2, y2, x3, y3)
        c = perp_dist(px, py, x3, y3, x1, y1)

        triangle_penalty = cd.exp(-a - b - c)
        triangle_cost = inside * triangle_penalty * 0.1  # 0.1 is a tunable weight
        # Velocity alignment with exponential weighting
        velocity_x = u_prediction * cd.cos(psi_prediction) - v_prediction * cd.sin(psi_prediction)
        velocity_y = u_prediction * cd.sin(psi_prediction) + v_prediction * cd.cos(psi_prediction)
        
        velocity_alignment = cd.sum1(
            velocity_x * desired_dir_x + velocity_y * desired_dir_y
        )
        velocity_cost = -w_velocity * (1 - cd.exp(-k_vel * velocity_alignment))
        
        # # Smooth control cost
        # control_cost = w_control * (
        #     cd.sum1(cd.exp(cd.diff(self.control_variable[:, 0])**2) - 1) +  # Propeller changes
        #     cd.sum1(cd.exp(cd.diff(self.control_variable[:, 1])**2) - 1)    # Rudder changes
        # )
        
        # Combine all costs
        final_cost = (
            cd.sum1(goal_cost) +      # Goal-seeking term
            # cd.sum1(escape_cost) +    # Pursuer avoidance term
            # velocity_cost +           # Velocity alignment term
            triangle_cost +           # Triangle avoidance term          
            total_cost               # Obstacle avoidance term
        )
        print(f"Final cost shape: {final_cost.shape}")

        return final_cost
   

        
 
        
    def nlpsolve_with_cost(self, Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos,return_cost=False):
       
        n0 = 1* np.ones(self.P)  # Initial propeller speed at minimum

        u0= 0.1*np.ones(self.P)
        
        x0 = np.concatenate([n0, u0])  # Default initial guess
        #x0 = np.concatenate([n0, u0])
        #print(f"Initial guess shape: {x0.shape}")
        
        # Bounds
        lbx_prop = [3] * self.P                    
        ubx_prop = [4] * self.P                  
        lbx_rudder = [-.61 ] * self.P    
        ubx_rudder = [.61 ] * self.P     
        
        # Combine bounds
        lbx = np.concatenate([lbx_prop, lbx_rudder])
        ubx = np.concatenate([ubx_prop, ubx_rudder])
        #print(f"Bounds shapes - lbx: {lbx.shape}, ubx: {ubx.shape}")
        
        self.g = []
        self.lbg = []
        self.ubg = []
        
    
        f = self.evader_cost(Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos)
        #print(f"Cost shape: {f.shape}")
        
        # Convert constraints to vectors
        g = cd.vertcat(*self.g) if self.g else cd.SX.zeros(0, 1)
        lbg = np.array(self.lbg)
        ubg = np.array(self.ubg)
        
        nlp = {
            'x': self.opt_var,  # Use the optimization variable defined in __init__
            'f': f,
            'g': g
        }  
        
        # Set solver options
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 1000,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.mu_strategy':'adaptive',
            'ipopt.warm_start_init_point': 'yes'
        }
        
        
            #print("Creating solver...")
        solver = cd.nlpsol('solver', 'ipopt', nlp, opts)
            #print("Solver created successfully")
            
            #print("Solving NLP...")
        solved = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
            #print("NLP solved successfully")
            
            # Extract solution
        solution = np.array(solved['x']).flatten()
        self.prev_solution = solution
        n_prop = solution[:self.P]  
        rudder = solution[self.P:]
        if np.any(np.isnan(n_prop)) or np.any(np.isnan(rudder)):
            raise ValueError("NaN values  in nlp_slovein optimization solution")
        control = np.column_stack([n_prop, rudder])
       
    
       
    
    # Debug print final control
        print(f"Final of evader  control commands:")
        print(f"Propeller evader: {control[:,0]}")
        print(f"Rudder evader(deg): {control[:,1]}")
        if return_cost: 
         optimal_cost=float(solved['f'])
            
         return control,optimal_cost
        return control
