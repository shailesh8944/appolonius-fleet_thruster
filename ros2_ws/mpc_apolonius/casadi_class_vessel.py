import casadi as ca
import numpy as np
from typing import Dict, Optional, List

from scipy.integrate import solve_ivp

import sys
sys.path.append('workspaces/mavlab/ros2_ws/src/mpc_apolonius')
from casadi_module_kinematics import Smat, eul_to_rotm, eul_rate_matrix


from module_dynamics import coriolis_matrix

from typing import Dict
import casadi as ca
import numpy as np

class Vessel:
    """
    CasADi symbolic vessel model for marine dynamics.
    State vector x = [u, v, w, p, q, r, x, y, z, phi, theta, psi, act_port, act_stbd]
    Control vector u = [act_port_cmd, act_stbd_cmd] in [-1, 1].
    """

    def __init__(self, vessel_params: Dict, hydrodynamic_data: Dict,ros_flag: bool = False):
        self.ros_flag = ros_flag
        self.params = vessel_params
        self.hydro = hydrodynamic_data
        self.dt = vessel_params['time_step']
        self.Tmax = vessel_params['sim_time']
        self.g = vessel_params['g']
        self.rho = vessel_params['rho'] 
        self.L = vessel_params['L']
        self.U = vessel_params['U']
        
        # Mass parameters
        self.mass = vessel_params['mass']
        self.cog = vessel_params['cog']
        self.gyration = np.array(vessel_params['gyration'])

        # Generate mass matrix
        self._generate_mass_matrix()

        # Dimensionalization flag
        self.dim_flag = hydrodynamic_data.get('dim_flag', False) 
        

        # Extract hydrodynamic coefficients
        self.X_u = hydrodynamic_data.get('X_u', 0.0)
        self.X_ud = hydrodynamic_data.get('X_ud', 0.0)
        self.Y_v = hydrodynamic_data.get('Y_v', 0.0)
        self.Y_vd = hydrodynamic_data.get('Y_vd', 0.0)
        self.Y_r = hydrodynamic_data.get('Y_r', 0.0)
        self.Y_rd = hydrodynamic_data.get('Y_rd', 0.0)
        self.N_v = hydrodynamic_data.get('N_v', 0.0)
        self.N_vd = hydrodynamic_data.get('N_vd', 0.0)
        self.N_r = hydrodynamic_data.get('N_r', 0.0)
        self.N_rd = hydrodynamic_data.get('N_rd', 0.0)
        self.thrust_scale = float(vessel_params.get('thrust_scale', 5.0))
        self.thruster_lever = float(vessel_params.get('thruster_lever', 0.21))
        self.T_act = float(vessel_params.get('T_act', 0.2))

        # Dimensionalize if needed
        if not self.dim_flag:
            self._dimensionalize_coefficients(self.rho, self.L, self.U)
        
        # Initialize state (symbolic)
        initial_velocity = ca.DM(vessel_params.get('initial_velocity', [0.0]*6))
        initial_position = ca.DM(vessel_params.get('initial_position', [0.0]*6))
        initial_act = ca.DM([0.0, 0.0])
        
        # Combine into state vector
        self.current_state = ca.vertcat(initial_velocity, initial_position, initial_act)
        
        # Convert CasADi DM to numpy array properly for history
        self.initial_state = np.array(self.current_state.full()).squeeze()
        
        # Initialize history with proper numpy array
        num_steps = int(self.Tmax / self.dt) + 2
        self.history = np.zeros((num_steps, len(self.initial_state)))
        # Store initial state in history with proper conversion
        self.history[0, :] = self.initial_state  # Now using numpy array
        
        self.time_index = 1
        self.t = 0.0
        self.control_type = vessel_params['control_type']
        # Create symbolic variables
        self.x_sym = ca.SX.sym('x', 14, 1)
        self.u_sym = ca.SX.sym('u', 2, 1)

        # Initialize current state (numerical)
        
        # Precompute symbolic ODE
        self.xdot_sym = self.vessel_ode(self.x_sym, self.u_sym)

        # Create CasADi integrator
        self.integrator = self.create_integrator(self.xdot_sym, self.x_sym, self.u_sym, self.dt)
        

    # -----------------------------
    # Mass and added mass matrices
    # -----------------------------
    def _dimensionalize_coefficients(self, rho, L, U):
        #====================================================================================
        # TODO: Code this function to dimenionalize non-dimensional hydrodynamic derivatives
        #====================================================================================
        
        # Write your code here

        
        #====================================================================================
        
        """Convert non-dimensional coefficients to dimensional form"""
        # Surge coefficients
        self.X_u *= 0.5 * rho * L**2 * U
        self.X_ud *= 0.5 * rho * L**3
        
        # Sway coefficients 
        self.Y_v *= 0.5 * rho * L**2 * U
        self.Y_vd *= 0.5 * rho * L**3
        self.Y_r *= 0.5 * rho * L**3 * U
        self.Y_rd *= 0.5 * rho * L**4

        # Yaw coefficients
        self.N_v *= 0.5 * rho * L**3 * U
        self.N_vd *= 0.5 * rho * L**4
        self.N_r *= 0.5 * rho * L**4 * U
        self.N_rd *= 0.5 * rho * L**5

        # Thruster coefficients are handled via thrust_scale and thruster_lever.

    def _generate_mass_matrix(self):
        #=======================================================================================
        # TODO: Code this function to generate the mass matrix and store it in self.mass_matrix
        #=======================================================================================
        self.mass_matrix = np.zeros((6,6))

        # Write your code here
        m = self.mass
        x_G, y_G, z_G = self.cog  
        I_xx, I_yy, I_zz = self.gyration ** 2 * m  

    
        self.mass_matrix = np.array([[m, 0, 0, 0, m * z_G, -m * y_G],
        [0, m, 0, -m * z_G, 0, m * x_G],
        [0, 0, m, m * y_G, -m * x_G, 0],
        [0, -m * z_G, m * y_G, I_xx, 0, 0],
        [m * z_G, 0, -m * x_G, 0, I_yy, 0],
        [-m * y_G, m * x_G, 0, 0, 0, I_zz]])
        


    def added_mass_matrix(self):
        M_A = ca.SX.zeros(6,6)
        M_A[0,0] = -self.hydro['X_ud']
        M_A[1,1] = -self.hydro['Y_vd']
        M_A[1,5] = -self.hydro['Y_rd']
        M_A[5,1] = -self.hydro['N_vd']
        M_A[5,5] = -self.hydro['N_rd']
        return M_A

    # -----------------------------
    # Coriolis forces
    # -----------------------------
    def coriolis_forces(self, M, vel):
        u=vel[0]
        v=vel[1]
        w=vel[2]
        p=vel[3]
        q=vel[4]
        r=vel[5]
        C = ca.SX.zeros(6,6)
        # Only simple diagonal Coriolis terms for illustration
        C[0,1] = -M[2,2]*r; C[0,2] = M[1,1]*q
        F_c = -ca.mtimes(C, vel[0:6])
        return F_c

    # -----------------------------
    # Hydrodynamics
    # -----------------------------
    def hydrodynamic_forces(self, vel):
        u=vel[0]
        v=vel[1]
        w=vel[2]
        p=vel[3]
        q=vel[4]
        r=vel[5]
        F = ca.SX.zeros(6,1)
        F[0] = self.hydro['X_u']*(u - self.params['U'])
        F[1] = self.hydro['Y_v']*v + self.hydro['Y_r']*r
        F[5] = self.hydro['N_v']*v + self.hydro['N_r']*r
        return F

    def thruster_forces(self, act_port, act_stbd):
        act_port = ca.fmin(ca.fmax(act_port, -1.0), 1.0)
        act_stbd = ca.fmin(ca.fmax(act_stbd, -1.0), 1.0)
        T_port = self.thrust_scale * act_port
        T_stbd = self.thrust_scale * act_stbd
        tau = ca.SX.zeros(6, 1)
        tau[0] = T_port + T_stbd
        tau[5] = self.thruster_lever * (T_stbd - T_port)
        return tau

    # -----------------------------
   
    # -----------------------------
    def vessel_ode(self, state, control):
        """CasADi vessel ODE reduced to 3-DOF dynamics, full 14-state output."""
        # Keep full state layout for compatibility:
        # [u, v, w, p, q, r, x, y, z, phi, theta, psi, act_port, act_stbd]
        u = state[0]
        v = state[1]
        r = state[5]
        psi = state[11]
        act_port = state[12]
        act_stbd = state[13]

        # Initialize full derivative vector
        state_dot = ca.SX.zeros(14, 1)

        # Compute forces from existing models and keep only 3-DOF channels (X, Y, N)
        vel6 = ca.vertcat(state[0], state[1], state[2], state[3], state[4], state[5])
        F_hyd = self.hydrodynamic_forces(vel6)
        F_thrust = self.thruster_forces(act_port, act_stbd)
        F3 = ca.vertcat(
            F_hyd[0] + F_thrust[0],  # X
            F_hyd[1] + F_thrust[1],  # Y
            F_hyd[5] + F_thrust[5],  # N
        )

        # Reduced inertia matrix for (u, v, r) from full (u, v, w, p, q, r)
        M_RB = self.mass_matrix
        M_A = self.added_mass_matrix()
        M = M_RB + M_A
        M3 = ca.vertcat(
            ca.horzcat(M[0, 0], M[0, 1], M[0, 5]),
            ca.horzcat(M[1, 0], M[1, 1], M[1, 5]),
            ca.horzcat(M[5, 0], M[5, 1], M[5, 5]),
        )

        accel3 = ca.mtimes(ca.inv(M3), F3)
        state_dot[0] = accel3[0]  # u_dot
        state_dot[1] = accel3[1]  # v_dot
        state_dot[5] = accel3[2]  # r_dot

        # 3-DOF kinematics in NED plane
        state_dot[6] = u * ca.cos(psi) - v * ca.sin(psi)  # x_dot
        state_dot[7] = u * ca.sin(psi) + v * ca.cos(psi)  # y_dot
        state_dot[11] = r  # psi_dot

        # Keep other states present but frozen in reduced model:
        # w,p,q,z,phi,theta derivatives remain zero.
        # Thruster actuator dynamics
        act_port_cmd = ca.fmin(ca.fmax(control[0], -1.0), 1.0)
        act_stbd_cmd = ca.fmin(ca.fmax(control[1], -1.0), 1.0)
        state_dot[12] = (act_port_cmd - act_port) / self.T_act
        state_dot[13] = (act_stbd_cmd - act_stbd) / self.T_act
        return state_dot

    # -----------------------------
    # CasADi integrator (one-step)
    # -----------------------------
    
    
        
    # def create_integrator(self, xdot, x, u, dt):
    #     """
    #     Create an RK4 step as a CasADi Function: xf = RK4(x,u)
    #     xdot is the symbolic rhs (self.xdot_sym) or will be computed by vessel_ode.
    #     """
    #     # Build an ODE evaluator (returns a vector)
    #     f = ca.Function('f', [x, u], [xdot])   # good: f(x,u)[0] is the vector

    #     # RK4 stages (each f(...) returns a 1-element list; take [0])
    #     k1 = f(x, u)[0]
    #     k2 = f(x + (dt/2) * k1, u)[0]
    #     k3 = f(x + (dt/2) * k2, u)[0]
    #     k4 = f(x + dt * k3, u)[0]

    #     # RK4 update
    #     x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    #     # Return a callable integrator
    #     return ca.Function('rk4_integrator', [x, u], [x_next], ['x0', 'p'], ['xf'])

    # # -----------------------------
    # Step forward
    
    def create_integrator(self, xdot, x, u, dt):
        """Create CasADi integrator with correct options"""
        # Define ODE dictionary
        ode = {
            'x': x,    # State
            'p': u,    # Control parameters
            'ode': xdot # Right hand side of ODE
        }
        
        # Integrator options for CVODES
        opts = {
            'tf': dt,           # Final time
            'abstol': 1e-3,     # Absolute tolerance
            'reltol': 1e-3,     # Relative tolerance
            'max_num_steps': 50,  # Maximum number of steps
            'print_stats': False
        }
        
        # Create and return integrator
        return ca.integrator('integrator', 'cvodes', ode, opts)
        # -----------------------------
    # def step(self, delta_c_val):
    #     """Step forward in time using CasADi integrator"""
    #     # Handle symbolic inputs
    #     if isinstance(self.current_state, (ca.SX, ca.MX)) or isinstance(delta_c_val, (ca.SX, ca.MX)):
    #         # For symbolic computation, just return the integrated state
    #         sol = self.integrator(x0=self.current_state, p=delta_c_val)
    #         return sol['xf']
    #     else:
    #         try:
    #             # For numeric computation
    #             sol = self.integrator(x0=self.current_state, p=delta_c_val)
                
    #             # Convert CasADi DM to numpy safely
    #             if isinstance(sol['xf'], ca.DM):
    #                 self.current_state = np.array(sol['xf'].full()).flatten()
    #             else:
    #                 self.current_state = sol['xf']
                
    #             # Store in history
    #             self.history[self.time_index, :] = self.current_state
    #             self.time_index += 1
    #             self.t += self.dt
                
    #             return self.current_state
                
    #         except Exception as e:
    #             print(f"Error in step method: {e}")
    #             print(f"current_state type: {type(self.current_state)}")
    #             print(f"delta_c_val type: {type(delta_c_val)}")
    #             raise 


    def step(self, control_val):
        """
        Advance one dt. If control_val or current_state is symbolic return symbolic xf.
        If numeric, convert x0/p -> DM, call integrator and convert DM -> numpy.
        """
        # Symbolic path: keep everything symbolic
        if isinstance(control_val, (ca.SX, ca.MX)) or isinstance(self.current_state, (ca.SX, ca.MX)):
            sol = self.integrator(x0=self.current_state, p=control_val)
            # return SX/MX next state (used during MPC construction)
            return sol['xf']

        # Numeric path: convert inputs to DM so integrator returns DM (no implicit SX->numpy)
        x0_dm = ca.DM(self.current_state)        # ensures integrator sees DM
        p_dm = ca.DM(control_val)
        sol = self.integrator(x0=x0_dm, p=p_dm)  # sol['xf'] is DM

        # convert DM to numpy safely
        xf_dm = sol['xf']
        try:
            self.current_state = np.array(xf_dm.full()).flatten()
        except Exception:
            # fallback for 1-D DM
            self.current_state = np.array(xf_dm).flatten()

        # advance time and save history
        self.t += self.dt
        if not getattr(self, "ros_flag", False):
            self.history[self.time_index, :] = self.current_state
            self.time_index += 1

        return self.current_state

    # -----------------------------
    # Reset vessel
    # -----------------------------
    def reset(self):
        self.current_state = self.initial_state.copy()
        self.t = 0.0
        self.time_index = 1
        self.history[:] = 0
        self.history[0, :] = self.current_state

    # -----------------------------
    # Simulate vessel
    # -----------------------------
    def simulate(self):
        """Simulate the vessel"""
        
        self.reset()
        while self.t < self.Tmax:
            self.step()
        
        # Trim history array to actual size
        self.history = self.history[:self.time_index, :]
