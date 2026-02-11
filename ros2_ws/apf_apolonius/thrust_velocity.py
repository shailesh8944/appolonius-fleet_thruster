import numpy as np
from module_kinematics import ssa


def compute_local_polar(pursuer_pos, evader_pos):
    """
    Returns distance and bearing-like alpha in pursuer-local frame.
    Kept for your debugging/telemetry.
    """
    dx = evader_pos[0] - pursuer_pos[0]
    dy = evader_pos[1] - pursuer_pos[1]
    dist = np.hypot(dx, dy)
    alpha = np.arctan2(dy, dx)
    return dist, alpha


def allocate_forces_to_thrusters(Fx, Mz, lever=0.26):
    """
    Solve:
        [Fx] = [ 1   1 ] [T_port]
        [Mz]   [-l  +l ] [T_stbd]

    NOTE: lever must match your thruster y-offset magnitude (0.26 m).
    """
    TA = np.array([[1.0, 1.0],
                   [-lever, lever]], dtype=float)
    F = np.array([[Fx],
                  [Mz]], dtype=float)
    T = np.linalg.pinv(TA) @ F
    return float(T[0, 0]), float(T[1, 0])  # (port, stbd)


def pwm_to_act(pwm):
    pwm = float(np.clip(pwm, 1400.0, 1600.0))
    return (pwm - 1500.0) / 100.0


def force_to_pwm_to_act(thrust):
    """
    Your experimental thrust->PWM fits (same as your PID code).
    Returns actuator cmd in [-1,1].
    """
    thrust = float(thrust)

    if thrust < -0.05:
        pwm = 8.394 * thrust**2 + 118.290 * thrust + 1544.198
        return float(np.clip(pwm_to_act(pwm), -1.0, 1.0))
    elif thrust > 0.05:
        pwm = -5.419 * thrust**2 + 93.808 * thrust + 1455.677
        return float(np.clip(pwm_to_act(pwm), -1.0, 1.0))
    else:
        return 0.0


class ThrustVelocityController:
    """
    Velocity (vx,vy) -> desired heading -> smooth heading -> PD yaw moment -> thruster allocation -> actuator commands.
    Adds:
      - desired-heading low-pass (angle-safe)
      - yaw moment saturation
      - optional deadband
    """

    def __init__(
        self,
        lever=0.21,
        F_const=5.0, 
        Kp_psi=1.35,
        Kd_r=0.35,
        mz_max=1.2,
        hdg_deadband_deg=2.0,
        hdg_tau=0.12,
        max_hdg_rate_deg=160.0,
        verbose=False
    ):
        self.lever = float(lever)
        self.F_const = float(F_const)
        self.Kp_psi = float(Kp_psi)
        self.Kd_r = float(Kd_r)
        self.mz_max = float(mz_max)
        self.hdg_deadband = np.deg2rad(float(hdg_deadband_deg))
        self.hdg_tau = float(hdg_tau)
        self.max_hdg_rate = np.deg2rad(float(max_hdg_rate_deg))
        self.verbose = bool(verbose)
        # Per-vessel desired-heading memory to avoid cross-coupling between pursuers.
        self._hdg_ref_mem = {}

    def velocity_to_controls(self, velocity, pursuer_state, evader_state, vessel_id="default", dt=0.1):
        """
        pursuer_state expected layout (as in your main_thrust_mpc):
            u at [0], v at [1], yaw_rate r at [5], x at [6], y at [7], yaw psi at [11]
        """
        vx, vy = float(velocity[0]), float(velocity[1])

        pursuer_pos = np.asarray(pursuer_state[6:8], dtype=float)
        evader_pos = np.asarray(evader_state[6:8], dtype=float)
        _, _alpha_local = compute_local_polar(pursuer_pos, evader_pos)  # optional

        current_heading = float(pursuer_state[11])   # ψ
        yaw_rate = float(pursuer_state[5])           # r
        dt = float(max(dt, 1e-3))

        # 1) desired heading from velocity
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            desired_heading = ssa(current_heading) 
        else:
            desired_heading = float(np.arctan2(vy, vx))

        # 2) desired-heading filtering (per pursuer) with angle-safe rate limiting.
        prev_ref = self._hdg_ref_mem.get(vessel_id, current_heading)
        # First-order low-pass in angle space
        alpha = dt / max(self.hdg_tau + dt, 1e-6)
        ref_step = alpha * ssa(desired_heading - prev_ref)
        desired_heading_lp = ssa(prev_ref + ref_step)
        # Additional hard heading-reference slew bound
        max_step = self.max_hdg_rate * dt
        dref = np.clip(ssa(desired_heading_lp - prev_ref), -max_step, max_step)
        desired_heading_f = ssa(prev_ref + dref)
        self._hdg_ref_mem[vessel_id] = desired_heading_f

        # 3) heading error
        heading_error = ssa(desired_heading_f - current_heading)

        # deadband to prevent micro-oscillation
        if abs(heading_error) < self.hdg_deadband:
            heading_error = 0.0

        # 4) PD yaw moment (P on heading error, D on yaw rate)
        # Gain scheduling: stronger proportional yaw action for larger heading errors.
        kp_scale = 1.0 + 0.8 * min(abs(heading_error) / np.deg2rad(90.0), 1.0)
        # IMPORTANT: use wrapped error directly (not sin(error)) so large heading
        # errors keep strong turning authority.
        Mz = (self.Kp_psi * kp_scale) * heading_error - self.Kd_r * yaw_rate
        Mz = float(np.clip(Mz, -self.mz_max, self.mz_max))

        # 5) adaptive surge force:
        # only mildly reduce forward push while turning hard.
        dist_to_evader = float(np.linalg.norm(evader_pos - pursuer_pos))
        turn_factor = 1.0 - 0.15 * min(abs(heading_error) / np.deg2rad(90.0), 1.0)
        near_factor = 1.0
        Fx = float(self.F_const * turn_factor * near_factor)

        # 6) allocate -> thrusts
        T_port, T_stbd = allocate_forces_to_thrusters(Fx, Mz, lever=self.lever)
        T_port = float(np.clip(T_port, -1.0, 1.0))
        T_stbd = float(np.clip(T_stbd, -1.0, 1.0))

        # 7) thrust -> actuator
        port_act = force_to_pwm_to_act(T_port)
        stbd_act = force_to_pwm_to_act(T_stbd)

        if self.verbose:
            print(
                f"[VEL→THR] des={np.degrees(desired_heading):.1f}deg "
                f"des_f={np.degrees(desired_heading_f):.1f}deg "
                f"cur={np.degrees(current_heading):.1f}deg "
                f"err={np.degrees(heading_error):.1f}deg "
                f"Fx={Fx:.2f} Mz={Mz:.2f} dist={dist_to_evader:.2f} "
                f"T_port={T_port:.2f} T_stbd={T_stbd:.2f} "
                f"act_port={port_act:.2f} act_stbd={stbd_act:.2f}"
            )

        return port_act, stbd_act


# Backward-compatible wrapper (so you don’t break other code)
_default_controller = ThrustVelocityController(verbose=False)

def velocity_to_controls1(velocity, pursuer_state, evader_state, lever=0.21, F_const=5.0, dt=0.1, **_kwargs):
    _default_controller.lever = float(lever)
    _default_controller.F_const = float(F_const)
    vessel_id = _kwargs.get('vessel_id', "default")
    return _default_controller.velocity_to_controls(
        velocity, pursuer_state, evader_state, vessel_id=vessel_id, dt=float(dt)
    )
