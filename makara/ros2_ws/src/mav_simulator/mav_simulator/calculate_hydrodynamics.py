"""
File: calculate_hydrodynamics.py
Description: This file contains the CalculateHydrodynamics class which is responsible for
             computing hydrodynamic forces and coefficients for marine vessels.
             
             Key functionalities include:
             - Generating mass matrices for vessel dynamics
             - Calculating added mass from hydrodynamic data files
             - Computing cross-flow drag for different vessel types
             - Implementing Hoerner's method for hydrodynamic coefficient estimation
             - Supporting various hydrodynamic modeling approaches for marine vessels
             
             This module is used by the Vessel class to obtain the hydrodynamic properties
             needed for accurate simulation of vessel motion in water.
             
Author: MAV Simulator Team
"""
import sys
import numpy as np
import json
from mav_simulator.module_kinematics import Smat
from mav_simulator.terminalMessages import print_info


class CalculateHydrodynamics:
    def __init__(self):
        self.grid = None
        self.length = None
        self.U_des = None
        self.ode_options = {}
        self.rho = None
    def _generate_mass_matrix(self,CG,mass,gyration):
    
        """Generate the mass matrix"""
        # Calculate gyration tensor about vessel frame
        xprdct2 = np.diag(gyration)**2 - Smat(CG)@Smat(CG)
        
        # Generate the inertia matrix (3 x 3)using radii of gyration
        inertia_matrix = xprdct2 * mass

        # Generate the mass matrix
        mass_matrix = np.zeros((6,6))
        mass_matrix[0:3][:, 0:3] = mass * np.eye(3)
        mass_matrix[3:6][:, 3:6] = inertia_matrix
        mass_matrix[0:3][:, 3:6] = -Smat(CG) * mass
        mass_matrix[3:6][:, 0:3] = Smat(CG) * mass

        return mass_matrix

    def calculate_added_mass_from_hydra(self, hydra_file):
        """Calculate added mass coefficients from HydRA data file.
        
        Args:
            hydra_file (str): Path to the HydRA JSON file containing hydrodynamic data
            
        Returns:
            np.ndarray: 6x6 added mass matrix
        """

    

        # Load hydrodynamic data from file
        with open(hydra_file, 'r') as file:
            mdict = json.load(file)
        
        # Extract arrays from data
        omg = np.array(mdict['w'])      # Frequency array
        AM = np.array(mdict['AM'])      # Added mass array
        BD = np.array(mdict['BD'])      # Damping array
        M = np.array(mdict['M'])        # Mass matrix
        C = np.array(mdict['C'])        # Restoring matrix

        # Extract zero-frequency added mass and specific components
        A_zero = AM[1, :, :, 0, 0]      # Zero frequency added mass
        A33 = AM[1:, 2, 2, 0, 0]        # Heave added mass
        A44 = AM[1:, 3, 3, 0, 0]        # Roll added mass
        A55 = AM[1:, 4, 4, 0, 0]        # Pitch added mass

        # Calculate natural frequencies through iteration
        # Heave natural frequency
        wn3_old = 0
        wn3_new = np.sqrt(C[2, 2] / (M[2, 2] + A33[0]))
        while np.abs(wn3_old - wn3_new) > 1e-6:
            wn3_old = wn3_new
            wn3_new = np.sqrt(C[2, 2] / (M[2, 2] + np.interp(wn3_old, omg[1:], A33)))
        wn3 = wn3_new

        # Roll natural frequency
        wn4_old = 0
        wn4_new = np.sqrt(C[3, 3] / (M[3, 3] + A44[0]))
        while np.abs(wn4_old - wn4_new) > 1e-6:
            wn4_old = wn4_new
            wn4_new = np.sqrt(C[3, 3] / (M[3, 3] + np.interp(wn4_old, omg[1:], A44)))
        wn4 = wn4_new

        # Pitch natural frequency
        wn5_old = 0
        wn5_new = np.sqrt(C[4, 4] / (M[4, 4] + A55[0]))
        while np.abs(wn5_old - wn5_new) > 1e-6:
            wn5_old = wn5_new
            wn5_new = np.sqrt(C[4, 4] / (M[4, 4] + np.interp(wn5_old, omg[1:], A55)))
        wn5 = wn5_new

        # Interpolate added mass at natural frequencies
        A33_wn = np.interp(wn3, omg[1:], A33)
        A44_wn = np.interp(wn4, omg[1:], A44)
        A55_wn = np.interp(wn5, omg[1:], A55)

        # Construct final added mass matrix (converting from ENU to NED)
        A = A_zero.copy()
        A[2:5][:, 2:5] = np.diag(np.array([A33_wn, A44_wn, A55_wn]))

        # used for linear damping
        B33_wn = np.interp(wn3, omg[1:], BD[1:, 2, 2, 0, 0])
        B44_wn = np.interp(wn4, omg[1:], BD[1:, 3, 3, 0, 0])
        B55_wn = np.interp(wn5, omg[1:], BD[1:, 4, 4, 0, 0])
        
        # Change from ENU to NED
        R = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])
        
        A[0:3,0:3] = R @ A[0:3,0:3] @ R.T
        A[3:6,3:6] = R @ A[3:6,3:6] @ R.T

        return A
    
    def cross_flow_drag(self):
        """Calculate cross-flow drag coefficients for surface ships.
        
        This method calculates the hydrodynamic coefficients for 
        sway-yaw motions using strip theory and Hoerner's cross-flow drag formulation.
        """
        Cd_2D = self.hoerner()
        x = self.grid.x_sec
        T = self.grid.T_sec

        v = np.linspace(-2, 2, 100)         # Sway velocity [m/s]
        r = np.linspace(-1.4, 1.4, 100)     # Yaw velocity [rad/s] - max value corresponds to turning radius of ~ 1.5L

        v_grid, r_grid = np.meshgrid(v, r)
        v_grid = v_grid.flatten()
        r_grid = r_grid.flatten()

        v_plus_xr_grid = v_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + r_grid[:, np.newaxis] @ x[np.newaxis, :]
        Y_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x)
        N_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x)

        A = np.zeros((np.size(Y_grid), 4))
        b = np.zeros((np.size(Y_grid), 2))

        A[:, 0] = v_grid * np.abs(v_grid)
        A[:, 1] = v_grid * np.abs(r_grid)
        A[:, 2] = r_grid * np.abs(v_grid)
        A[:, 3] = r_grid * np.abs(r_grid)

        b[:, 0] = Y_grid
        b[:, 1] = N_grid
        
        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        self.ode_options['Y_v_av'] = quad_coeff[0, 0]
        self.ode_options['Y_v_ar'] = quad_coeff[1, 0]
        self.ode_options['Y_r_av'] = quad_coeff[2, 0]
        self.ode_options['Y_r_ar'] = quad_coeff[3, 0]
        self.ode_options['N_v_av'] = quad_coeff[0, 1]
        self.ode_options['N_v_ar'] = quad_coeff[1, 1]
        self.ode_options['N_r_av'] = quad_coeff[2, 1]
        self.ode_options['N_r_ar'] = quad_coeff[3, 1]

        Rn = self.U_des * self.length * 1e6
        Cf = 0.075 / ((np.log10(Rn) - 2) ** 2)
        Cr = 0.0
        k = 0.0
        Ct = Cr + Cf * (1 + k)

        self.ode_options['X_u_au'] = -self.grid.WettedArea * Ct

        for key, value in self.ode_options.items():
            if key.startswith(("X_", "Y_", "N_")):
                print(f"{key}: {value}")

    def cross_flow_drag_AUV(self):
        """Calculate cross-flow drag coefficients for an AUV.
        
        This method calculates the hydrodynamic coefficients for both
        sway-yaw and heave-pitch motions using strip theory and Hoerner's
        cross-flow drag formulation.
        """
        if not all([self.grid, self.length, self.U_des]):
            raise ValueError("grid, length, and U_des must be set before calling this method")
            
        Cd_2D = self.hoerner()
        x = self.grid.x_sec
        T = self.grid.T_sec

        v = np.linspace(-2, 2, 100)         # Sway velocity [m/s]
        r = np.linspace(-1.4, 1.4, 100)     # Yaw velocity [rad/s]
        w = np.linspace(-2, 2, 100)         # Heave velocity [m/s]
        q = np.linspace(-1.4, 1.4, 100)     # Pitch velocity [rad/s]

        v_grid, r_grid = np.meshgrid(v, r)
        v_grid = v_grid.flatten()
        r_grid = r_grid.flatten()

        w_grid, q_grid = np.meshgrid(w, q)
        w_grid = w_grid.flatten()
        q_grid = q_grid.flatten()

        v_plus_xr_grid = v_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + r_grid[:, np.newaxis] @ x[np.newaxis, :]
        Y_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x)
        N_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * v_plus_xr_grid * np.abs(v_plus_xr_grid), x)

        w_plus_xr_grid = w_grid[:, np.newaxis] @ np.ones(np.size(x))[np.newaxis, :] + q_grid[:, np.newaxis] @ x[np.newaxis, :]
        Z_grid = - np.trapz(T[np.newaxis, :] * Cd_2D[np.newaxis, :] * w_plus_xr_grid * np.abs(w_plus_xr_grid), x)
        M_grid = - np.trapz(T[np.newaxis, :] * x[np.newaxis, :] * Cd_2D[np.newaxis, :] * w_plus_xr_grid * np.abs(w_plus_xr_grid), x)

        # For sway-yaw coefficients
        A = np.zeros((np.size(Y_grid), 4))
        b = np.zeros((np.size(Y_grid), 2))

        A[:, 0] = v_grid * np.abs(v_grid)
        A[:, 1] = v_grid * np.abs(r_grid)
        A[:, 2] = r_grid * np.abs(v_grid)
        A[:, 3] = r_grid * np.abs(r_grid)

        b[:, 0] = Y_grid
        b[:, 1] = N_grid
        
        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        # Store sway-yaw coefficients
        self.ode_options['Y_v_av'] = quad_coeff[0, 0]
        self.ode_options['Y_v_ar'] = quad_coeff[1, 0]
        self.ode_options['Y_r_av'] = quad_coeff[2, 0]
        self.ode_options['Y_r_ar'] = quad_coeff[3, 0]
        self.ode_options['N_v_av'] = quad_coeff[0, 1]
        self.ode_options['N_v_ar'] = quad_coeff[1, 1]
        self.ode_options['N_r_av'] = quad_coeff[2, 1]
        self.ode_options['N_r_ar'] = quad_coeff[3, 1]

        # For heave-pitch coefficients
        A = np.zeros((np.size(Z_grid), 4))
        b = np.zeros((np.size(Z_grid), 2))

        A[:, 0] = w_grid * np.abs(w_grid)
        A[:, 1] = w_grid * np.abs(q_grid)
        A[:, 2] = q_grid * np.abs(w_grid)
        A[:, 3] = q_grid * np.abs(q_grid)

        b[:, 0] = Z_grid
        b[:, 1] = M_grid

        quad_coeff, residue, rank, sing_val = np.linalg.lstsq(A, b, rcond=-1)

        # Store heave-pitch coefficients
        self.ode_options['Z_w_aw'] = quad_coeff[0, 0]
        self.ode_options['Z_w_aq'] = quad_coeff[1, 0]
        self.ode_options['Z_q_aw'] = quad_coeff[2, 0]
        self.ode_options['Z_q_aq'] = quad_coeff[3, 0]
        self.ode_options['M_w_aw'] = quad_coeff[0, 1]
        self.ode_options['M_w_aq'] = quad_coeff[1, 1]
        self.ode_options['M_q_aw'] = quad_coeff[2, 1]
        self.ode_options['M_q_aq'] = quad_coeff[3, 1]

        Rn = self.U_des * self.length * 1e6
        Cf = 0.075 / ((np.log10(Rn) - 2) ** 2)
        Cr = 0.0
        k = 0.0
        Ct = Cr + Cf * (1 + k)

        self.ode_options['X_u_au'] = -self.grid.WettedArea * Ct

        for key, value in self.ode_options.items():
            if key.startswith(("X_", "Y_", "N_", "M_", "Z_")):
              print(f"{key}: {value}")

    def hoerner(self):

        B_by_2T = self.grid.B_sec / 2 / self.grid.T_sec

        Cd_data = np.array([[0.0108623, 1.96608], 
            [0.176606, 1.96573],
            [0.353025, 1.89756],
            [0.451863, 1.78718],
            [0.472838, 1.58374],
            [0.492877, 1.27862],
            [0.493252, 1.21082],
            [0.558473, 1.08356],
            [0.646401, 0.998631],
            [0.833589, 0.87959],
            [0.988002, 0.828415],
            [1.30807, 0.759941],
            [1.63918, 0.691442],
            [1.85998, 0.657076],
            [2.31288, 0.630693],
            [2.59998, 0.596186],
            [3.00877, 0.586846],
            [3.45075, 0.585909],
            [3.7379, 0.559877],
            [4.00309, 0.559315]])

        Cd_linear = np.interp(B_by_2T, Cd_data[:, 0], Cd_data[:, 1])
        Cy_2d = np.zeros_like(B_by_2T)
        Cy_2d = np.where(B_by_2T <= 4.00309, Cd_linear, 0.559315)
        return Cy_2d