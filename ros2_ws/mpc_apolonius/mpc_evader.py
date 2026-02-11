import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from casadi_module_kinematics import ssa
from math import sin,cos,sqrt,pi,exp    
from CASadi_kcs_ode import simulation  # Import the MMG3 dynamics

from Evader_escape_strategy import compute_evader_heading_ramana




class controller():  
    def __init__(self, time_step, NP, NC, Q, r, evasion_weight=50000.0, evasion_length=10.0):
        self.time_step = time_step
        self.P = NP  # Prediction horizon
        self.NC = NC  # Control horizon
        self.Q = Q   # Cost weights
        self.r = r   # Obstacle radius
        self.w_evasion = evasion_weight
        self.evasion_length = evasion_length
        self.current_state = None
        self.thruster_var = cd.SX.sym('thruster', 2, self.P)
        self.opt_var = cd.reshape(self.thruster_var, 2 * self.P, 1)
        
    
    def prediction_model(self, states, thruster_cmd, h):
     """
     Predicts ship's future states using MMG3 dynamics
     
     """

     # Use dual-thruster actuation in [-1, 1]
     if isinstance(thruster_cmd, (cd.SX, cd.MX)):
        control = cd.vertcat(
            cd.fmin(cd.fmax(thruster_cmd[0], -1.0), 1.0),
            cd.fmin(cd.fmax(thruster_cmd[1], -1.0), 1.0)
        )
     else:
        arr = np.asarray(thruster_cmd).astype(float).flatten()
        if arr.size < 2:
            raise ValueError("Thruster command must have 2 elements (port, stbd).")
        control = np.clip(arr[:2], -1.0, 1.0)

     new_states = simulation(states, control, h)
     return new_states

    def _safe_theta(self, pursuer_state, evader_state):
        """
        Helper to compute theta_i with clipping for numerical stability.
        """
        try:
            evader_speed = float(evader_state[0])
            pursuer_speed = float(pursuer_state[0])
            if abs(evader_speed) < 1e-6:
                return 0.0
            ratio = np.clip(pursuer_speed / evader_speed, -0.9999, 0.9999)
            return 2.0 * np.arcsin(ratio)
        except Exception:
            print("[MPC] Warning: Invalid speeds for theta computation, defaulting to 0.")
            return 0.0

    def _compute_reference_heading(self, pursuer_states, evader_state):
        """
        Use cooperative controller utilities to compute desired psi_e.
        """
        try:
            psi_e = compute_evader_heading_ramana(pursuer_states, evader_state)
            if np.isnan(psi_e):
                raise ValueError("psi_e evaluated to NaN")
            return float(psi_e)
        except Exception as exc:
            print(f"[MPC] Falling back to evader heading. Reason: {exc}")
            return float(evader_state[11])

    def evader_cost(self, Xe, goal_evader, desired_heading, pursuer_states, t):
        """
        MPC stage cost that keeps the evader pointed along psi_e and moving to its goal.
        """
        h = t[1] - t[0]
        predicted_state = Xe
        cost = 0
        w_goal_dist = self.Q[0]
        w_goal_align = self.Q[1]
        w_heading_track = self.Q[2]
        w_thrust = 0.01
        w_evasion = self.w_evasion
        d_scale = max(self.evasion_length, 1.0)
        psi_target = float(desired_heading)

        for k in range(self.P):
            thruster_cmd = self.thruster_var[:, k]
            predicted_state = self.prediction_model(predicted_state, thruster_cmd, h)

            x_e = predicted_state[6]
            y_e = predicted_state[7]
            psi_e = predicted_state[11]

            dx_goal = goal_evader[0] - x_e
            dy_goal = goal_evader[1] - y_e
            dist_goal = cd.sqrt(dx_goal**2 + dy_goal**2)
            desired_goal_heading = cd.atan2(dy_goal, dx_goal)

            heading_goal_error = ssa(desired_goal_heading - psi_e)
            heading_track_error = ssa(psi_target - psi_e)

            # Exponential repulsion from pursuers (stronger when closer).
            evasion_cost = 0
            for pursuer in pursuer_states:
                dx_p = x_e - pursuer[6]
                dy_p = y_e - pursuer[7]
                dist_p = cd.sqrt(dx_p**2 + dy_p**2 + 1e-6)
                evasion_cost += cd.exp(-dist_p / d_scale)

            cost += (
                w_goal_dist * dist_goal
                + w_goal_align * heading_goal_error**2
                + w_heading_track * heading_track_error**2
                + w_evasion * evasion_cost
                + w_thrust * cd.sumsqr(thruster_cmd)
            )

        return cost
  
        
        
        
   

   
    def nlpsolve_with_cost(self, Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos):
        #print("\n=== Starting nlpsolve_with_cost ===")
        
        u0 = np.zeros(2 * self.P)
        if hasattr(self, 'prev_solution'):
         x0 = self.prev_solution if len(self.prev_solution) == 2 * self.P else u0.copy()
        else:
         x0 = u0.copy()
        
        # Bounds
                     
        lbx = [-1.0] * (2 * self.P)
        ubx = [1.0] * (2 * self.P)
        
        self.g = []
        self.lbg = []
        self.ubg = []
        
        pursuer_states = [Xp1, Xp2, Xp3]
        psi_reference = self._compute_reference_heading(pursuer_states, Xe)
       
        f = self.evader_cost(Xe, goal_evader, psi_reference, pursuer_states, t)
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
            'ipopt.max_iter': 500,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.mu_strategy': 'adaptive',
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.mu_strategy':'adaptive',
            'ipopt.warm_start_init_point': 'yes'
        }
        
        
            #print("Creating solver...")
        solver = cd.nlpsol('solver', 'ipopt', nlp, opts)
            #print("Solver created successfully")
            
            #print("Solving NLP...")
        solved = solver(x0=x0, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)
        thruster_seq = np.array(solved['x']).flatten().reshape(2, self.P)
        predicted_states = []
        current_state = Xe.copy()
        
        for k in range(self.P):
            next_state = self.prediction_model(
                current_state,
                thruster_seq[:, k],
                t[1]-t[0]
            )
            predicted_states.append(next_state)
            current_state = next_state.copy()
        
        # Extract positions for visualization
        predicted_path = np.array([[state[6], state[7]] for state in predicted_states])
        self.prev_solution = thruster_seq.reshape(-1)
        return thruster_seq[:, 0], predicted_path
