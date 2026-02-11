import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from casadi_module_kinematics import ssa
from math import sin,cos,sqrt,pi,exp    
from CASadi_kcs_ode import simulation  # Import the MMG3 dynamics

from Evader_escape_strategy import compute_evader_heading_ramana




class controller():  
    def __init__(self, time_step, NP, NC, Q, r, evasion_weight=250 ,evasion_length=3.0):
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
        self.T_act_est = 0.2
        self.act_hat = np.zeros(2, dtype=float)
        self.act_hat_initialized = False
        # Minimum mean thrust command to avoid spin-in-place behavior.
        self.min_forward_cmd = 0.30
        # Cap differential thrust to prevent violent yaw oscillation.
        self.max_diff_cmd = 0.55
        # Hard slew-rate limit for each thruster command between prediction steps.
        self.max_slew_cmd = 0.12
        # Yaw command shaping for straighter tracking.
        self.heading_deadband_deg = 6.0
        self.turn_tanh_gain = 0.9
        self.k_turn_cmd = 0.55
        self.k_r_cmd = 0.55
        # Near-goal command shaping to reduce final yaw oscillation / looping.
        self.goal_slow_radius = 4.0
        self.goal_diff_zero_radius = 2.0
        # First-step smoothing reference (last applied command).
        self.prev_applied_cmd = np.array([0.35, 0.35], dtype=float)
        # Keep a known-feasible warm-start / fallback sequence.
        self.last_successful_cmd = self.prev_applied_cmd.copy()
        self.last_successful_solution = np.tile(self.prev_applied_cmd.reshape(2, 1), (1, self.P)).reshape(-1, order='F')
        self.prev_solution = self.last_successful_solution.copy()
        self.last_cmd_source = "init"
        # Ramana-goal blend state (outer guidance fused into NMPC cost reference).
        self.beta_escape = 0.0
        self.beta_alpha = 0.25
        self.threat_d_in = 4.0
        self.threat_d_out = 8.0
        # Keep some goal influence even in high-threat mode to avoid orbit lock.
        self.beta_escape_max = 0.85
        # Escape-heading temporal filtering to reduce fast reference flips.
        self.psi_escape_filt = None
        self.psi_escape_alpha = 0.30
        self.psi_escape_rate_max_deg = 75.0
        # Direct separation objective (meters, cost gain).
        self.evade_d_safe = 5.0
        self.w_evade_sep = float(self.w_evasion)
        # IPOPT options for direct NMPC solve each cycle.
        self._solver_opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 500,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.mu_strategy': 'adaptive',
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.warm_start_init_point': 'yes'
        }
        self._solver_build_id = 0
        self.debug_mpc = False
        self.debug_every = 1
        self._debug_iter = 0
        self.last_debug = {}

    @staticmethod
    def _heading_error_to_goal(state, goal_evader):
        """Compute wrapped heading error (goal_heading - psi) for numeric states."""
        st = np.asarray(state, dtype=float).flatten()
        gx = float(goal_evader[0])
        gy = float(goal_evader[1])
        goal_heading = np.arctan2(gy - st[7], gx - st[6])
        psi = float(st[11])
        err = np.arctan2(np.sin(goal_heading - psi), np.cos(goal_heading - psi))
        return err, goal_heading, psi

    def _augment_state_for_nmpc(self, state):
        """
        Convert input state to 14D by appending internal actuator-state estimate
        when only odometry-like states are provided.
        """
        st = np.asarray(state, dtype=float).flatten()
        if st.size >= 14:
            out = st[:14].copy()
            self.act_hat = out[12:14].copy()
            self.act_hat_initialized = True
            return out
        if st.size >= 12:
            if not self.act_hat_initialized:
                self.act_hat = np.zeros(2, dtype=float)
                self.act_hat_initialized = True
            return np.concatenate([st[:12], self.act_hat.copy()])
        raise ValueError("State must contain at least 12 elements for NMPC.")

    def _update_actuator_observer(self, cmd):
        """
        First-order lag observer for actuator states based on applied command.
        """
        u = np.clip(np.asarray(cmd, dtype=float).flatten()[:2], -1.0, 1.0)
        dt = float(self.time_step)
        T = max(float(self.T_act_est), 1e-6)
        self.act_hat += (dt / T) * (u - self.act_hat)
        self.act_hat = np.clip(self.act_hat, -1.0, 1.0)
        
    
    def prediction_model(self, states, thruster_cmd, h):
        """
        Predicts ship's future states using MMG3 dynamics.
        """
        if isinstance(states, (cd.SX, cd.MX)):
            state_vec = states
        else:
            state_vec = np.asarray(states).flatten()
            if state_vec.size >= 14:
                state_vec = state_vec[:14]
            elif state_vec.size >= 12:
                state_vec = state_vec[:12]

        if isinstance(thruster_cmd, (cd.SX, cd.MX)):
            act_port = cd.fmin(cd.fmax(thruster_cmd[0], -1.0), 1.0)
            act_stbd = cd.fmin(cd.fmax(thruster_cmd[1], -1.0), 1.0)
            control = cd.vertcat(act_port, act_stbd)
        else:
            arr = np.asarray(thruster_cmd).astype(float).flatten()
            if arr.size < 2:
                raise ValueError("Thruster command must have 2 elements (port, stbd).")
            control = np.clip(arr[:2], -1.0, 1.0)

        new_states = simulation(state_vec, control, h)
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
    
    @staticmethod
    def _blend_heading(goal_heading, escape_heading, beta):
        """
        Circular blend of two headings with weight beta in [0,1].
        beta=0 -> pure goal, beta=1 -> pure escape.
        """
        b = float(np.clip(beta, 0.0, 1.0))
        sx = (1.0 - b) * np.cos(goal_heading) + b * np.cos(escape_heading)
        sy = (1.0 - b) * np.sin(goal_heading) + b * np.sin(escape_heading)
        return float(np.arctan2(sy, sx))

    @staticmethod
    def _nearest_pursuer_distance(evader_state, pursuer_states):
        xe, ye = float(evader_state[6]), float(evader_state[7])
        dmin = np.inf
        for p in pursuer_states:
            px, py = float(p[6]), float(p[7])
            dmin = min(dmin, np.hypot(px - xe, py - ye))
        return float(dmin if np.isfinite(dmin) else 1e9)

    def _update_escape_blend(self, dmin):
        """
        Distance-based threat blending with hysteresis-like smoothing.
        Closer pursuers -> larger beta (more Ramana heading influence).
        """
        din = float(self.threat_d_in)
        dout = float(self.threat_d_out)
        bmax = float(np.clip(self.beta_escape_max, 0.0, 1.0))
        if dmin <= din:
            beta_target = bmax
        elif dmin >= dout:
            beta_target = 0.0
        else:
            beta_target = bmax * (dout - dmin) / max(dout - din, 1e-6)
        a = float(np.clip(self.beta_alpha, 0.0, 1.0))
        self.beta_escape = (1.0 - a) * self.beta_escape + a * beta_target
        self.beta_escape = float(np.clip(self.beta_escape, 0.0, 1.0))
        return self.beta_escape

    def _filter_escape_heading(self, psi_raw, dt):
        """
        Low-pass + slew limit on escape heading to avoid zig-zag from
        rapidly switching Ramana active pairs.
        """
        psi_raw = float(psi_raw)
        dt = float(max(dt, 1e-3))
        if self.psi_escape_filt is None:
            self.psi_escape_filt = psi_raw
            return float(self.psi_escape_filt)

        # Angle-safe LPF update.
        alpha = float(np.clip(self.psi_escape_alpha, 0.0, 1.0))
        dpsi = float(ssa(psi_raw - self.psi_escape_filt))
        dpsi_lp = alpha * dpsi

        # Additional hard slew bound per control step.
        max_step = np.deg2rad(float(self.psi_escape_rate_max_deg)) * dt
        dpsi_lim = float(np.clip(dpsi_lp, -max_step, max_step))
        self.psi_escape_filt = float(ssa(self.psi_escape_filt + dpsi_lim))
        return float(self.psi_escape_filt)

    @staticmethod
    def _planar_velocity_from_state(st):
        """
        Convert body-frame (u,v,psi) into world-frame planar velocity.
        """
        u = float(st[0])
        v = float(st[1])
        psi = float(st[11])
        vx = u * np.cos(psi) - v * np.sin(psi)
        vy = u * np.sin(psi) + v * np.cos(psi)
        return vx, vy

    def _predict_path_numerical(self, Xe_aug, thruster_seq, h):
        """
        Roll out NMPC model numerically for visualization/diagnostics.
        """
        xk = np.asarray(Xe_aug, dtype=float).flatten()[:14]
        xy = []
        h = float(max(h, 1e-3))
        for k in range(self.P):
            uk = np.asarray(thruster_seq[:, k], dtype=float).flatten()[:2]
            xk_next = self.prediction_model(xk, uk, h)
            try:
                xk = np.array(xk_next, dtype=float).flatten()
            except Exception:
                xk = np.array(xk_next.full(), dtype=float).flatten()
            if xk.size >= 8:
                xy.append([float(xk[6]), float(xk[7])])
        return np.array(xy, dtype=float) if len(xy) else np.zeros((0, 2))

    def evader_cost(self, Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos, psi_escape_ref, beta_escape):
        """
        Single source of NMPC objective/constraints (no solver-pack duplication).
        """
        h = float(max(t[1] - t[0], 1e-3))
        xinit = self._augment_state_for_nmpc(Xe)

        w_dist = 20.0
        w_heading = 1200.0
        w_thrust = 0.01
        w_turn = 500.0
        w_yaw_rate = 220.0
        w_forward = 120.0
        w_reverse = 2000.0
        w_smooth = 450.0
        w_diff_rate = 450.0
        u_cmd_ref = 0.35
        err_db = np.deg2rad(self.heading_deadband_deg)
        b = float(np.clip(beta_escape, 0.0, 1.0))
        psi_escape_const = float(psi_escape_ref)

        p1vx, p1vy = self._planar_velocity_from_state(Xp1)
        p2vx, p2vy = self._planar_velocity_from_state(Xp2)
        p3vx, p3vy = self._planar_velocity_from_state(Xp3)

        self.g = []
        self.lbg = []
        self.ubg = []

        total_cost = 0
        goal_heading_error = 0.0

        for k in range(self.P):
            thruster_cmd = self.thruster_var[:, k]
            newstate = self.prediction_model(xinit, thruster_cmd, h)
            xinit = newstate

            x_e = newstate[6]
            y_e = newstate[7]
            psi_e = newstate[11]
            r_e = newstate[5]

            dx_goal = float(goal_evader[0]) - x_e
            dy_goal = float(goal_evader[1]) - y_e
            dist_to_goal = cd.sqrt(dx_goal**2 + dy_goal**2)
            goal_heading = cd.atan2(dy_goal, dx_goal)
            psi_ref = cd.atan2(
                (1.0 - b) * cd.sin(goal_heading) + b * np.sin(psi_escape_const),
                (1.0 - b) * cd.cos(goal_heading) + b * np.cos(psi_escape_const)
            )
            goal_heading_error = ssa(psi_ref - psi_e)
            heading_error_eff = cd.if_else(cd.fabs(goal_heading_error) < err_db, 0.0, goal_heading_error)
            goal_cost = w_dist * dist_to_goal + w_heading * heading_error_eff**2

            tk = (k + 1) * h
            p10x = float(Xp1[6]) + p1vx * tk
            p10y = float(Xp1[7]) + p1vy * tk
            p20x = float(Xp2[6]) + p2vx * tk
            p20y = float(Xp2[7]) + p2vy * tk
            p30x = float(Xp3[6]) + p3vx * tk
            p30y = float(Xp3[7]) + p3vy * tk
            d1 = cd.sqrt((x_e - p10x)**2 + (y_e - p10y)**2 + 1e-6)
            d2 = cd.sqrt((x_e - p20x)**2 + (y_e - p20y)**2 + 1e-6)
            d3 = cd.sqrt((x_e - p30x)**2 + (y_e - p30y)**2 + 1e-6)
            dmin_pred = cd.fmin(d1, cd.fmin(d2, d3))
            sep_gap = cd.fmax(0.0, float(self.evade_d_safe) - dmin_pred)
            sep_cost = float(self.w_evade_sep) * sep_gap**2 + 2.0 / (dmin_pred + 0.2)

            desired_diff = -self.k_turn_cmd * cd.tanh(self.turn_tanh_gain * heading_error_eff)
            actual_diff = thruster_cmd[1] - thruster_cmd[0]
            turn_cost = w_turn * (actual_diff - desired_diff)**2

            if k == 0:
                prev_u = cd.DM(self.prev_applied_cmd)
                prev_diff = float(self.prev_applied_cmd[1] - self.prev_applied_cmd[0])
            else:
                prev_u = self.thruster_var[:, k - 1]
                prev_diff = self.thruster_var[1, k - 1] - self.thruster_var[0, k - 1]

            smooth_cost = w_smooth * cd.sumsqr(thruster_cmd - prev_u)
            diff_rate_cost = w_diff_rate * (actual_diff - prev_diff)**2
            desired_r = self.k_r_cmd * heading_error_eff
            yaw_rate_cost = w_yaw_rate * (r_e - desired_r)**2

            cmd_mean = 0.5 * (thruster_cmd[0] + thruster_cmd[1])
            forward_cost = w_forward * (cmd_mean - u_cmd_ref)**2
            reverse_cost = w_reverse * cd.fmax(0.0, -cmd_mean)**2

            self.g.append(cmd_mean)
            self.lbg.append(self.min_forward_cmd)
            self.ubg.append(1.0)

            self.g.append(actual_diff)
            self.lbg.append(-self.max_diff_cmd)
            self.ubg.append(self.max_diff_cmd)

            for j in range(2):
                du = thruster_cmd[j] - prev_u[j]
                self.g.append(du)
                self.lbg.append(-self.max_slew_cmd)
                self.ubg.append(self.max_slew_cmd)

            total_cost += (
                w_thrust * cd.sumsqr(thruster_cmd)
                + goal_cost
                + sep_cost
                + turn_cost
                + smooth_cost
                + diff_rate_cost
                + yaw_rate_cost
                + forward_cost
                + reverse_cost
            )

        total_cost += 3000.0 * (goal_heading_error**2)
        return total_cost

    
    def nlpsolve_with_cost(self, Xe, Xp1, Xp2, Xp3, goal_evader, t, obs_pos, psi_reference=None):
        Xe_aug = self._augment_state_for_nmpc(Xe)
        h_raw = float(max(t[1] - t[0], 1e-3))

        if hasattr(self, 'prev_solution') and len(self.prev_solution) == 2 * self.P:
            x0 = np.asarray(self.prev_solution, dtype=float).copy()
        else:
            x0 = np.asarray(self.last_successful_solution, dtype=float).copy()

        pursuer_states = [Xp1, Xp2, Xp3]
        if psi_reference is None:
            psi_reference = self._compute_reference_heading(pursuer_states, Xe_aug)
        else:
            psi_reference = float(psi_reference)
        psi_reference = self._filter_escape_heading(psi_reference, h_raw)

        dmin = self._nearest_pursuer_distance(Xe_aug, pursuer_states)
        beta_escape = self._update_escape_blend(dmin)
        heading_err, goal_heading, psi_now = self._heading_error_to_goal(Xe_aug, goal_evader)
        # When threat is moderate/far and escape heading strongly opposes goal heading,
        # reduce escape weight to avoid orbiting around pursuer cluster.
        if dmin > float(self.threat_d_in):
            ge_sep = abs(float(ssa(psi_reference - goal_heading)))
            ge_sep_deg = float(np.degrees(ge_sep))
            if ge_sep_deg > 100.0:
                scale = float(np.clip(1.0 - 0.55 * (ge_sep_deg - 100.0) / 80.0, 0.45, 1.0))
                beta_escape *= scale

        psi_ref_now = self._blend_heading(goal_heading, psi_reference, beta_escape)
        heading_err_ref_now = float(ssa(psi_ref_now - psi_now))

        if self.debug_mpc:
            print(
                "[MPC] heading_err(deg), psi(deg), goal_heading(deg):",
                f"{np.degrees(heading_err):.2f}, {np.degrees(psi_now):.2f}, {np.degrees(goal_heading):.2f}"
            )
            print(
                "[MPC] beta_escape, dmin, psi_ref(deg), psi_ramana(deg):",
                f"{beta_escape:.2f}, {dmin:.2f}, {np.degrees(psi_ref_now):.2f}, {np.degrees(psi_reference):.2f}"
            )

        goal_flat = np.asarray(goal_evader, dtype=float).flatten()
        J = self.evader_cost(
            Xe_aug, Xp1, Xp2, Xp3, goal_evader, np.array([0.0, h_raw], dtype=float),
            obs_pos, psi_reference, beta_escape
        )
        g_sym = cd.vertcat(*self.g) if len(self.g) else cd.SX.zeros(0, 1)
        nlp = {'x': self.opt_var, 'f': J, 'g': g_sym}
        self._solver_build_id += 1
        solver = cd.nlpsol(
            f'solver_ev_{self._solver_build_id}',
            'ipopt',
            nlp,
            self._solver_opts
        )

        lbx = np.array([-1.0] * (2 * self.P), dtype=float)
        ubx = np.array([1.0] * (2 * self.P), dtype=float)
        solved = solver(
            x0=x0,
            lbg=np.array(self.lbg, dtype=float),
            ubg=np.array(self.ubg, dtype=float),
            lbx=lbx,
            ubx=ubx,
        )
        solver_stats = solver.stats()
        solve_ok = bool(solver_stats.get('success', False))
        solve_status = str(solver_stats.get('return_status', 'UNKNOWN'))
        solve_iter = int(solver_stats.get('iter_count', -1))
        cmd_source = "solver"

        sol_vec = np.array(solved['x'], dtype=float).flatten()
        thruster_seq_candidate = np.clip(sol_vec, -1.0, 1.0).reshape(2, self.P, order='F')
        if solve_ok:
            thruster_seq = thruster_seq_candidate
            self.last_successful_solution = thruster_seq.reshape(-1, order='F').copy()
            self.last_successful_cmd = thruster_seq[:, 0].copy()
        else:
            # Reject failed NLP outputs.
            # Prefer heading-aware feasible fallback over stale zero/non-reactive plans.
            heading_err_eff_now = 0.0 if abs(heading_err_ref_now) < np.deg2rad(self.heading_deadband_deg) else heading_err_ref_now
            desired_diff_fail = -self.k_turn_cmd * np.tanh(self.turn_tanh_gain * heading_err_eff_now)
            mean_fail = float(max(self.min_forward_cmd, 0.35))
            u_port = float(np.clip(mean_fail - 0.5 * desired_diff_fail, -1.0, 1.0))
            u_stbd = float(np.clip(mean_fail + 0.5 * desired_diff_fail, -1.0, 1.0))
            fallback_plan = np.tile(np.array([u_port, u_stbd], dtype=float).reshape(2, 1), (1, self.P))

            if hasattr(self, 'last_successful_solution') and len(self.last_successful_solution) == 2 * self.P:
                # Blend heading-aware fallback with last successful plan for smoothness.
                last_plan = np.asarray(self.last_successful_solution, dtype=float).reshape(2, self.P, order='F').copy()
                thruster_seq = 0.6 * fallback_plan + 0.4 * last_plan
                thruster_seq = np.clip(thruster_seq, -1.0, 1.0)
                cmd_source = "fallback_heading+last_success"
            else:
                thruster_seq = fallback_plan
                self.last_successful_solution = thruster_seq.reshape(-1, order='F').copy()
                self.last_successful_cmd = np.array([u_port, u_stbd], dtype=float)
                cmd_source = "fallback_heading"

        # Arrival shaping: taper yaw differential and blend mean thrust toward cruise floor
        # when close to the goal to avoid tiny end-game loops.
        d_goal_now_pre = float(np.hypot(goal_flat[0] - Xe_aug[6], goal_flat[1] - Xe_aug[7]))
        if d_goal_now_pre < float(self.goal_slow_radius):
            cmd_mean0 = float(0.5 * (thruster_seq[0, 0] + thruster_seq[1, 0]))
            cmd_diff0 = float(thruster_seq[1, 0] - thruster_seq[0, 0])

            a_mean = float(np.clip(d_goal_now_pre / max(float(self.goal_slow_radius), 1e-6), 0.0, 1.0))
            mean_target = float(max(self.min_forward_cmd, 0.30))
            cmd_mean_new = mean_target + a_mean * (cmd_mean0 - mean_target)

            a_diff = 1.0
            if d_goal_now_pre < float(self.goal_diff_zero_radius):
                a_diff = float(np.clip(d_goal_now_pre / max(float(self.goal_diff_zero_radius), 1e-6), 0.0, 1.0))
            cmd_diff_new = a_diff * cmd_diff0

            u_port = np.clip(cmd_mean_new - 0.5 * cmd_diff_new, -1.0, 1.0)
            u_stbd = np.clip(cmd_mean_new + 0.5 * cmd_diff_new, -1.0, 1.0)
            thruster_seq[0, 0] = u_port
            thruster_seq[1, 0] = u_stbd
            cmd_source = f"{cmd_source}+goal_shape"

        self._update_actuator_observer(thruster_seq[:, 0])
        self.prev_applied_cmd = thruster_seq[:, 0].copy()
        heading_err_eff_now = 0.0 if abs(heading_err_ref_now) < np.deg2rad(self.heading_deadband_deg) else heading_err_ref_now
        desired_diff_now = -self.k_turn_cmd * np.tanh(self.turn_tanh_gain * heading_err_eff_now)
        cmd_diff_now = float(thruster_seq[1, 0] - thruster_seq[0, 0])
        cmd_mean_now = float(0.5 * (thruster_seq[1, 0] + thruster_seq[0, 0]))

        if self.debug_mpc:
            print("thrusters at mpc ________________________evader command:",
                  thruster_seq[0, 0], thruster_seq[1, 0])
            print(
                "[MPC] desired_diff, cmd_diff, cmd_mean:",
                f"{desired_diff_now:.3f}, {cmd_diff_now:.3f}, {cmd_mean_now:.3f}"
            )

        predicted_path = self._predict_path_numerical(Xe_aug, thruster_seq, h_raw)

        # Lightweight numeric diagnostics for root-cause analysis.
        d_goal_now = float(np.hypot(goal_flat[0] - Xe_aug[6], goal_flat[1] - Xe_aug[7]))
        d_goal_end = d_goal_now
        if predicted_path.shape[0] > 0:
            d_goal_end = float(np.hypot(goal_flat[0] - predicted_path[-1, 0], goal_flat[1] - predicted_path[-1, 1]))
        pred_goal_progress = d_goal_now - d_goal_end

        p_states = [Xp1, Xp2, Xp3]
        p_vels = [self._planar_velocity_from_state(Xp1), self._planar_velocity_from_state(Xp2), self._planar_velocity_from_state(Xp3)]
        dmin_pred_min = np.inf
        if predicted_path.shape[0] > 0:
            for k in range(predicted_path.shape[0]):
                tk = (k + 1) * h_raw
                xk, yk = float(predicted_path[k, 0]), float(predicted_path[k, 1])
                for ps, pv in zip(p_states, p_vels):
                    px = float(ps[6]) + float(pv[0]) * tk
                    py = float(ps[7]) + float(pv[1]) * tk
                    dmin_pred_min = min(dmin_pred_min, float(np.hypot(xk - px, yk - py)))
        if not np.isfinite(dmin_pred_min):
            dmin_pred_min = float(dmin)

        self.last_debug = {
            'h_raw': float(h_raw),
            'h_used': float(h_raw),
            'obj': float(np.array(solved['f']).squeeze()),
            'd_goal_now': d_goal_now,
            'd_goal_end': d_goal_end,
            'pred_goal_progress': float(pred_goal_progress),
            'dmin_now': float(dmin),
            'dmin_pred_min': float(dmin_pred_min),
            'beta_escape': float(beta_escape),
            'heading_err_deg': float(np.degrees(heading_err_ref_now)),
            'psi_ref_deg': float(np.degrees(psi_ref_now)),
            'psi_ramana_deg': float(np.degrees(psi_reference)),
            'cmd_port_plan': float(thruster_seq[0, 0]),
            'cmd_stbd_plan': float(thruster_seq[1, 0]),
            'cmd_diff_plan': float(cmd_diff_now),
            'cmd_mean_plan': float(cmd_mean_now),
            'solve_ok': solve_ok,
            'solve_status': solve_status,
            'solve_iter': solve_iter,
            'cmd_source': cmd_source,
        }
        self._debug_iter += 1

        if self.debug_mpc and (self._debug_iter % max(int(self.debug_every), 1) == 0):
            dbg = self.last_debug
            print(
                "[MPCDBG]",
                f"h={dbg['h_used']:.2f}s raw={dbg['h_raw']:.2f}s",
                f"obj={dbg['obj']:.2f}",
                f"dgoal={dbg['d_goal_now']:.2f}->{dbg['d_goal_end']:.2f} (Δ={dbg['pred_goal_progress']:+.2f})",
                f"dmin={dbg['dmin_now']:.2f} pred_min={dbg['dmin_pred_min']:.2f}",
                f"beta={dbg['beta_escape']:.2f}",
                f"herr={dbg['heading_err_deg']:.1f}deg",
                f"src={dbg['cmd_source']}",
                f"cmd=({dbg['cmd_port_plan']:.3f},{dbg['cmd_stbd_plan']:.3f})",
                f"mean={dbg['cmd_mean_plan']:.3f} diff={dbg['cmd_diff_plan']:.3f}",
            )

        # Warm-start from last successful trajectory; do not poison with failed solver output.
        self.prev_solution = np.asarray(self.last_successful_solution, dtype=float).copy()
        return thruster_seq[:, 0], predicted_path
     
