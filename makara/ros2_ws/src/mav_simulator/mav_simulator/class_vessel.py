"""
File: class_vessel.py
Description: Vessel class for MAV simulator (6-DOF), updated with:
  - Thruster actuator model in [-1,1] using experimental PWM fits (act <-> thrust)
  - Correct yaw moment generation via r x F (differential thrust)
  - Added debug logs: act -> PWM -> thrust, per-thruster moments, total tau

Author: MAV Simulator Team
"""

from typing import Dict
import numpy as np
from scipy.integrate import solve_ivp
import pandas as pd

from mav_simulator.module_kinematics import (
    Smat,
    clip,
    eul_to_rotm,
    eul_rate_matrix,
    eul_to_quat,
    ssa,
    quat_to_eul,
    quat_to_rotm,
)
from mav_simulator.calculate_hydrodynamics import CalculateHydrodynamics
from mav_simulator.terminalMessages import print_info, print_warning


class Vessel:
    vessel_node = None
    delta_c = None
    n_c = None

    def __init__(self, vessel_params: Dict, hydrodynamic_data: Dict, vessel_id: int, ros_flag: bool = True):
        self.ros_flag = ros_flag

        self.active_dof = np.array(vessel_params['active_dof'], dtype=float)
        self.maintain_speed = vessel_params['maintain_speed']

        self.vessel_config = vessel_params
        self.vessel_name = vessel_params['name']
        self.vessel_id = vessel_id
        self.gps_datum = vessel_params['gps_datum']

        # Force direction mapping
        self.force_indices = {'X': 0, 'Y': 1, 'Z': 2, 'K': 3, 'M': 4, 'N': 5}

        # Base vessel parameters
        self.g = float(vessel_params['gravity'])
        self.rho = float(vessel_params['density'])
        self.L = float(vessel_params['geometry']['length'])
        self.B = float(vessel_params['geometry']['breadth'])
        self.D = float(vessel_params['geometry']['depth'])
        self.U = float(vessel_params.get('U', 0.0))

        self.coriolis_flag = hydrodynamic_data.get('coriolis_flag', False)

        # Mass parameters
        self.mass = float(vessel_params['inertia']['mass'])
        self.CG = np.array(vessel_params['geometry']['CG']['position'], dtype=float)
        self.gyration = np.array(vessel_params['geometry']['gyration'], dtype=float)

        if vessel_params['inertia']['inertia_matrix'] == "None":
            hydrodynamics = CalculateHydrodynamics()
            self.mass_matrix = hydrodynamics._generate_mass_matrix(self.CG, self.mass, self.gyration)
            print_info(f"Dimensionalized mass matrix: {self.mass_matrix}")
        else:
            self.mass_matrix = np.array(vessel_params['inertia']['inertia_matrix'], dtype=float)

        if vessel_params['inertia']['added_mass_matrix'] == "None":
            hydrodynamics = CalculateHydrodynamics()
            dim_A = hydrodynamics.calculate_added_mass_from_hydra(hydrodynamic_data['hydra_file'])
            self.added_mass_matrix = np.array(dim_A, dtype=float)
            print_info(f"Dimensionalized added mass matrix: {self.added_mass_matrix}")
            if not self.coriolis_flag:
                print_warning("Coriolis Matrix will not be used for Dynamics since flag set False")
        else:
            self.added_mass_matrix = np.array(vessel_params['inertia']['added_mass_matrix'], dtype=float)

        # Buoyancy parameters
        self.W = self.mass * self.g
        self.buoyancy_mass = float(vessel_params['inertia']['buoyancy_mass'])
        self.B_force = self.buoyancy_mass * self.g
        self.CB = np.array(vessel_params['geometry']['CB']['position'], dtype=float)

        # Dimensionalization flag
        self.dim_flag = hydrodynamic_data.get('dim_flag', False)

        # Hydrodynamics dictionary (force/moment polynomials)
        self.hydrodynamics = {}
        for coeff_name, coeff_value in hydrodynamic_data.items():
            if coeff_name in ('dim_flag', 'hydra_file', 'coriolis_flag'):
                continue
            self.hydrodynamics[coeff_name] = coeff_value
            setattr(self, coeff_name, coeff_value)

        if not self.dim_flag:
            self._dimensionalize_coefficients(self.rho, self.L, self.U)

        # NACA airfoil data (optional)
        try:
            self.naca_data = pd.read_csv(vessel_params['control_surfaces']['naca_file'])
        except Exception:
            self.naca_data = None
            print_warning("NACA airfoil data file path not specified; will use control surface hydrodynamics if specified")

        # Attitude representation
        self.use_quaternion = bool(vessel_params['initial_conditions'].get('use_quaternion', False))
        attitude_size = 4 if self.use_quaternion else 3

        # Control surfaces
        self.n_control_surfaces = 0
        self.control_surfaces = None
        try:
            if vessel_params.get('control_surfaces', None) is not None:
                self.control_surfaces = vessel_params['control_surfaces']
                self.n_control_surfaces = len(self.control_surfaces.get('control_surfaces', []))
                print_info(f"Control surfaces for vessel {self.vessel_name}: {self.n_control_surfaces}")
            else:
                print_warning("No control surfaces specified.")
        except Exception:
            print_warning("No control surfaces specified.")

        # Thrusters
        self.n_thrusters = 0
        self.thrusters = None
        try:
            if vessel_params.get('thrusters', None) is not None:
                self.thrusters = vessel_params['thrusters']
                self.n_thrusters = len(self.thrusters.get('thrusters', []))
                print_info(f"Thrusters for vessel {self.vessel_name}: {self.n_thrusters}")
            else:
                print_warning(f"No thrusters specified for {self.vessel_name}.")
        except Exception:
            print_warning(f"No thrusters specified for {self.vessel_name}.")
            self.thrusters = None
            self.n_thrusters = 0

        if self.n_thrusters == 0:
            print_warning(f"[{self.vessel_name}] n_thrusters is 0. Differential thrust/yaw will be disabled.")

        # Optional thrust scaling (simulation calibration knob)
        self.thrust_scale = float(vessel_params.get('thrust_scale', 1.0))

        # -----------------------------
        # NEW: Debug thrust logging control
        # -----------------------------
        self._thrust_log_count = 0
        self._thrust_log_limit = 50  # prints first 50 log lines only

        # Initial state vector
        initial_velocity = np.array(vessel_params['initial_conditions']['start_velocity'], dtype=float)
        initial_position = np.array(vessel_params['initial_conditions']['start_location'], dtype=float)
        initial_orientation = np.array(vessel_params['initial_conditions']['start_orientation'], dtype=float)

        if self.use_quaternion:
            initial_orientation = eul_to_quat(initial_orientation)

        initial_control = np.zeros(self.n_control_surfaces, dtype=float)
        initial_thruster = np.zeros(self.n_thrusters, dtype=float)  # thruster state = actuator in [-1,1]

        self.current_state = np.concatenate([
            initial_velocity, initial_position, initial_orientation,
            initial_control, initial_thruster
        ])
        self.initial_state = self.current_state.copy()
        self.current_state_der = np.zeros_like(self.current_state)

        self.state_size = len(self.current_state)

        self.Tmax = float(vessel_params['sim_time'])
        self.dt = float(vessel_params['time_step'])
        self.t = 0.0

        # Commanded values:
        self.delta_c = np.zeros(self.n_control_surfaces, dtype=float)
        self.n_c = np.zeros(self.n_thrusters, dtype=float)  # actuator commands in [-1,1]

        # History
        num_timesteps = int(self.Tmax / self.dt) + 2
        self.history = np.zeros((num_timesteps, self.state_size))
        self.history[0, :] = self.current_state
        self.time_index = 1

    # -----------------------------
    # Helpers for quaternion mode
    # -----------------------------
    @staticmethod
    def _quat_rate(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
        q0, q1, q2, q3 = q
        p, qv, r = omega
        qdot = 0.5 * np.array([
            [-q1*p - q2*qv - q3*r],
            [ q0*p + q2*r  - q3*qv],
            [ q0*qv - q1*r + q3*p ],
            [ q0*r  + q1*qv - q2*p ]
        ], dtype=float).ravel()
        return qdot

    # -----------------------------
    # Dimensionalize coefficients
    # -----------------------------
    def _dimensionalize_coefficients(self, rho, L, U):
        if self.dim_flag:
            return

        for coeff_name, coeff_value in list(self.hydrodynamics.items()):
            if coeff_value == 0 or '_' not in coeff_name:
                continue

            parts = coeff_name.split('_')
            if len(parts) < 2:
                continue

            force_dir = parts[0]
            components = parts[1:]

            if force_dir not in self.force_indices:
                continue

            factor = 0.5 * rho
            L_power = 2 if force_dir in ['X', 'Y', 'Z'] else 3

            for comp in components[1:]:
                if comp in ['p', 'q', 'r']:
                    L_power += 1

            factor *= L**L_power

            U_power = 0
            for comp in components:
                if comp in ['u', 'v', 'w', 'p', 'q', 'r']:
                    U_power += 1
            factor *= U**U_power

            dimensionalized_value = coeff_value * factor
            setattr(self, coeff_name, dimensionalized_value)
            self.hydrodynamics[coeff_name] = dimensionalized_value

    # -----------------------------
    # Main ODE
    # -----------------------------
    def vessel_ode(self, t, state):
        state = np.asarray(state, dtype=float)

        use_quaternion = self.use_quaternion
        attitude_size = 4 if use_quaternion else 3
        attitude_end = 9 + attitude_size

        control_start = attitude_end
        thruster_start = control_start + self.n_control_surfaces

        vel = state[0:6].copy()
        pos = state[6:9].copy()

        if use_quaternion:
            quat = state[9:13].copy()
            eul = quat_to_eul(quat)
            Rnb = quat_to_rotm(quat)
        else:
            eul = state[9:attitude_end].copy()
            Rnb = eul_to_rotm(eul)

        vel_eff = vel * self.active_dof

        F_hyd = self.hydrodynamic_forces(vel_eff)

        if self.n_control_surfaces > 0:
            delta_state = state[control_start:thruster_start]
            F_control = self.control_forces(delta_state, vel_eff)
        else:
            F_control = np.zeros(6)

        if self.n_thrusters > 0:
            thr_state = state[thruster_start:]
            F_thrust = self.thruster_forces(thr_state)
        else:
            F_thrust = np.zeros(6)

        F_g = self.gravitational_forces(eul[0], eul[1])

        M_RB = self.mass_matrix
        M_A = self.added_mass_matrix

        if self.coriolis_flag:
            C_RB, C_A = self.calculate_coriolis_matrices(vel_eff)

        if np.any(np.abs(M_A) > 2 * np.abs(M_RB)):
            M = M_RB
            if self.coriolis_flag:
                C_A = np.zeros_like(C_A)
        else:
            M = M_RB + M_A

        F_C = np.zeros(6)
        if self.coriolis_flag:
            F_C = (C_RB + C_A) @ vel_eff

        F_total = F_hyd + F_control + F_thrust - F_g - F_C

        state_dot = np.zeros_like(state)
        state_dot[0:6] = np.linalg.solve(M, F_total)

        if use_quaternion:
            state_dot[6:9] = Rnb @ vel_eff[0:3]
            state_dot[9:13] = self._quat_rate(quat, vel_eff[3:6])
        else:
            state_dot[6:9] = Rnb @ vel_eff[0:3]
            state_dot[9:attitude_end] = eul_rate_matrix(eul) @ vel_eff[3:6]

        if self.n_control_surfaces > 0:
            delta_c = np.array(self.delta_c, dtype=float)
            for i in range(self.n_control_surfaces):
                cs = self.control_surfaces['control_surfaces'][i]
                T = float(cs['control_surface_T'])
                delta_max = float(cs['control_surface_delta_max'])
                deltad_max = float(cs['control_surface_deltad_max'])

                delta_c[i] = np.clip(delta_c[i], -delta_max, delta_max)
                ddelta = (delta_c[i] - state[control_start + i]) / max(T, 1e-6)
                ddelta = np.clip(ddelta, -deltad_max, deltad_max)
                state_dot[control_start + i] = ddelta

        if self.n_thrusters > 0:
            act_cmd = np.clip(np.array(self.n_c, dtype=float), -1.0, 1.0)
            for i in range(self.n_thrusters):
                T_act = float(self.thrusters['thrusters'][i].get('T_act', 0.2))
                state_dot[thruster_start + i] = (act_cmd[i] - state[thruster_start + i]) / max(T_act, 1e-6)

        state_dot[0:6] = state_dot[0:6] * self.active_dof
        if not use_quaternion:
            state_dot[9:12] = state_dot[9:12] * self.active_dof[3:6]

        return state_dot

    # -----------------------------
    # Hydrodynamic forces
    # -----------------------------
    def hydrodynamic_forces(self, vel):
        u, v, w, p, q, r = vel
        F = np.zeros(6)

        if self.maintain_speed:
            vel_map = {'u': u - self.U, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r}
        else:
            vel_map = {'u': u, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r}

        for coeff_name, coeff_value in self.hydrodynamics.items():
            if coeff_value == 0:
                continue

            parts = coeff_name.split('_')
            if len(parts) < 2:
                continue

            force_dir = parts[0]
            if force_dir not in self.force_indices:
                continue

            force = float(coeff_value)
            for token in parts[1:]:
                if token.startswith('a') and len(token) > 1 and token[1:] in vel_map:
                    v_char = token[1:]
                    force *= abs(vel_map[v_char])
                elif token in vel_map:
                    force *= vel_map[token]

            F[self.force_indices[force_dir]] += force

        return F

    # -----------------------------
    # Control surface forces
    # -----------------------------
    def control_forces(self, delta, vel, stall_angle=15, Cl=None, Cd=None):
        tau = np.zeros(6)
        if self.control_surfaces is None or self.n_control_surfaces == 0:
            return tau

        u, v, w, p, q, r = vel

        for surface in self.control_surfaces['control_surfaces']:
            max_delta = np.deg2rad(surface['control_surface_delta_max'])
            sid = int(surface['control_surface_id']) - 1

            if surface.get('control_surface_hydrodynamics', 'None') != 'None':
                d = clip(ssa(delta[sid]), max_delta)
                for coeff_name, coeff_value in surface['control_surface_hydrodynamics'].items():
                    if 'delta' not in coeff_name:
                        continue
                    val = float(coeff_value)
                    if not self.dim_flag:
                        force_pow = {'X': 2, 'Y': 2, 'Z': 2, 'K': 3, 'M': 3, 'N': 3}
                        fd = coeff_name.split('_')[0]
                        factor = 0.5 * self.rho * self.L**force_pow[fd] * self.U**2
                        val *= factor
                    force_dir = coeff_name.split('_')[0]
                    if force_dir in self.force_indices:
                        tau[self.force_indices[force_dir]] += val * d
                continue

            if self.naca_data is None:
                continue

            sd = np.array(surface['control_surface_location'], dtype=float)
            area = float(surface['control_surface_area'])

            phi_s, theta_s, psi_s = surface['control_surface_orientation']
            cphi, sphi = np.cos(phi_s), np.sin(phi_s)
            cth, sth = np.cos(theta_s), np.sin(theta_s)
            cpsi, spsi = np.cos(psi_s), np.sin(psi_s)

            R = np.array([
                [cth*cpsi, -cth*spsi, sth],
                [sphi*sth*cpsi + cphi*spsi, -sphi*sth*spsi + cphi*cpsi, -sphi*cth],
                [-cphi*sth*cpsi + sphi*spsi, cphi*sth*spsi + sphi*cpsi, cphi*cth]
            ])

            netU = u
            netV = v + r*sd[0] + p*sd[2]
            netW = w + p*sd[1] - q*sd[0]

            V_surface = R.T @ np.array([netU, netV, netW], dtype=float)

            d = clip(ssa(delta[sid]), max_delta)
            alpha = np.arctan2(V_surface[2], V_surface[0]) + d
            V_mag = np.sqrt(V_surface[0]**2 + V_surface[2]**2)

            q_dyn = 0.5 * self.rho * V_mag**2 * area

            if abs(np.rad2deg(alpha)) > stall_angle:
                Lf = 0.0
                Df = q_dyn * 0.1
            else:
                if Cl is None or Cd is None:
                    alpha_deg = np.clip(np.rad2deg(alpha),
                                        self.naca_data['Alpha'].min(),
                                        self.naca_data['Alpha'].max())
                    Cl_i = np.interp(alpha_deg, self.naca_data['Alpha'], self.naca_data['Cl'])
                    Cd_i = np.interp(alpha_deg, self.naca_data['Alpha'], self.naca_data['Cd'])
                else:
                    Cl_i, Cd_i = Cl, Cd

                Lf = q_dyn * Cl_i
                Df = q_dyn * Cd_i

            F_surface = np.array([-Df, 0.0, -Lf], dtype=float)
            F_body = R @ F_surface

            tau[0:3] += F_body
            tau[3:6] += np.cross(sd, F_body)

        return tau

    # -----------------------------
    # Thruster: actuator [-1,1] -> thrust (PWM-fit)
    # -----------------------------
    def act_to_thrust(self, act: float) -> float:
        """
        Convert actuator command in [-1,1] to thrust using PWM fits.
        We also log: act -> pwm -> thrust.
        """
        act = float(np.clip(act, -1.0, 1.0))
        pwm = float(np.clip(1500.0 + 100.0 * act, 1400.0, 1600.0))

        if abs(act) < 1e-3:
            if self._thrust_log_count < self._thrust_log_limit:
                print_info(f"[{self.vessel_name}] act_to_thrust: act={act:+.3f} -> pwm={pwm:.1f} -> thrust=0.0 (deadband)")
                self._thrust_log_count += 1
            return 0.0

        # 16V T200 data (1400–1600 PWM) interpolated from uploaded table
        pwm_table = np.array(
            [1400, 1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432, 1436, 1440, 1444,
             1448, 1452, 1456, 1460, 1464, 1468, 1472, 1476, 1480, 1484, 1488, 1492,
             1496, 1500, 1504, 1508, 1512, 1516, 1520, 1524, 1528, 1532, 1536, 1540,
             1544, 1548, 1552, 1556, 1560, 1564, 1568, 1572, 1576, 1580, 1584, 1588,
             1592, 1596, 1600],
            dtype=float,
        )
        thrust_table = np.array(
            [-0.3175144, -0.2948348, -0.26761928, -0.24493968, -0.226796, -0.2041164,
             -0.17690088, -0.16329312, -0.14514944, -0.12246984, -0.10886208, -0.0907184,
             -0.07257472, -0.05896696, -0.0453592, -0.03628736, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.04082328, 0.05443104, 0.07257472, 0.09525432, 0.113398, 0.13154168,
             0.15422128, 0.17690088, 0.19958048, 0.22226008, 0.25401152, 0.27669112,
             0.30844256, 0.33112216, 0.36740952, 0.39462504],
            dtype=float,
        )
        thrust = float(np.interp(pwm, pwm_table, thrust_table))
        # Convert kgf (data) to N for dynamics
        thrust *= 9.80665

        if self._thrust_log_count < self._thrust_log_limit:
            print_info(f"[{self.vessel_name}] act_to_thrust: act={act:+.3f} -> pwm={pwm:.1f} -> thrust={thrust:+.4f}")
            self._thrust_log_count += 1

        return thrust

    def thruster_forces(self, act_vec: np.ndarray) -> np.ndarray:
        """
        Thruster forces/moments from actuator states in [-1,1].
        Force is applied along body +x, and moments are r x F.
        Also logs per thruster: act, pwm, thrust, moment_z, and total tau.
        """
        tau = np.zeros(6)
        if self.thrusters is None or self.n_thrusters == 0:
            return tau

        act_vec = np.clip(np.asarray(act_vec, dtype=float), -1.0, 1.0)

        if self._thrust_log_count < self._thrust_log_limit:
            print_info(f"[{self.vessel_name}] thruster_forces: act_vec={act_vec.tolist()}")

        for i, thruster in enumerate(self.thrusters['thrusters']):
            td = np.array(thruster['thruster_location'], dtype=float)  # [x,y,z]

            act_i = float(act_vec[i])
            pwm_i = float(np.clip(1500.0 + 100.0 * act_i, 1400.0, 1600.0))

            X_prop = float(self.thrust_scale * self.act_to_thrust(act_i))

            F_body = np.array([X_prop, 0.0, 0.0], dtype=float)
            tau[0:3] += F_body

            moment = np.cross(td, F_body)
            tau[3:6] += moment

            if self._thrust_log_count < self._thrust_log_limit:
                print_info(
                    f"[{self.vessel_name}] thr{i}: act={act_i:+.3f}, pwm={pwm_i:.1f}, "
                    f"Fx={X_prop:+.4f}, loc={td.tolist()}, moment={moment.tolist()}, moment_z={moment[2]:+.6f}"
                )
                self._thrust_log_count += 1

        if self._thrust_log_count < self._thrust_log_limit:
            print_info(f"[{self.vessel_name}] total_tau: Fx={tau[0]:+.4f}, Fy={tau[1]:+.4f}, Nz={tau[5]:+.6f}")
            self._thrust_log_count += 1

        return tau

    # -----------------------------
    # Integration loop
    # -----------------------------
    def step(self):
        sol = solve_ivp(self.vessel_ode, [self.t, self.t + self.dt], self.current_state, method='RK45')
        self.current_state = sol.y[:, -1]

        use_quaternion = self.use_quaternion
        att_size = 4 if use_quaternion else 3
        thruster_start = (9 + att_size) + self.n_control_surfaces
        if self.n_thrusters > 0:
            self.current_state[thruster_start:] = np.clip(self.current_state[thruster_start:], -1.0, 1.0)

        # Hard enforce surge speed if maintain_speed is enabled
        if self.maintain_speed:
            u = float(self.current_state[0])
            if abs(u) < 1e-6:
                self.current_state[0] = float(self.U)
            else:
                self.current_state[0] = float(np.sign(u) * self.U)

        self.current_state_der = self.vessel_ode(self.t + self.dt, self.current_state)
        self.t = sol.t[-1]

        if not self.ros_flag:
            self.history[self.time_index, :] = self.current_state
            self.time_index += 1

    def reset(self):
        self.current_state = self.initial_state.copy()
        self.t = 0.0
        self.time_index = 1
        self.history[0, :] = self.current_state

    def simulate(self):
        self.reset()
        while self.t < self.Tmax:
            self.step()
        self.history = self.history[:self.time_index, :]

    # -----------------------------
    # Gravity / buoyancy
    # -----------------------------
    def gravitational_forces(self, phi, theta):
        sth = np.sin(theta)
        cth = np.cos(theta)
        sphi = np.sin(phi)
        cphi = np.cos(phi)

        gvec = np.array([
            (self.W - self.B_force) * sth,
            -(self.W - self.B_force) * cth * sphi,
            -(self.W - self.B_force) * cth * cphi,
            -(self.CG[1]*self.W - self.CB[1]*self.B_force) * cth * cphi +
            (self.CG[2]*self.W - self.CB[2]*self.B_force) * cth * sphi,
            (self.CG[2]*self.W - self.CB[2]*self.B_force) * sth +
            (self.CG[0]*self.W - self.CB[0]*self.B_force) * cth * cphi,
            -(self.CG[0]*self.W - self.CB[0]*self.B_force) * cth * sphi -
            (self.CG[1]*self.W - self.CB[1]*self.B_force) * sth
        ], dtype=float)

        return gvec

    # -----------------------------
    # Coriolis matrices
    # -----------------------------
    def calculate_coriolis_matrices(self, vel):
        v1 = vel[0:3]
        v2 = vel[3:6]

        M_RB = self.mass_matrix
        M11 = M_RB[0:3, 0:3]
        M12 = M_RB[0:3, 3:6]
        M21 = M_RB[3:6, 0:3]
        M22 = M_RB[3:6, 3:6]

        C_RB = np.zeros((6, 6))
        C_RB[0:3, 3:6] = -Smat(M11 @ v1 + M12 @ v2)
        C_RB[3:6, 0:3] = -Smat(M11 @ v1 + M12 @ v2)
        C_RB[3:6, 3:6] = -Smat(M21 @ v1 + M22 @ v2)

        M_A = self.added_mass_matrix
        A11 = M_A[0:3, 0:3]
        A12 = M_A[0:3, 3:6]
        A21 = M_A[3:6, 0:3]
        A22 = M_A[3:6, 3:6]

        C_A = np.zeros((6, 6))
        C_A[0:3, 3:6] = -Smat(A11 @ v1 + A12 @ v2)
        C_A[3:6, 0:3] = -Smat(A11 @ v1 + A12 @ v2)
        C_A[3:6, 3:6] = -Smat(A21 @ v1 + A22 @ v2)

        return C_RB, C_A
