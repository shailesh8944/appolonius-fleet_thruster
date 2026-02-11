import numpy as np
from cooperative_pursuer import compute_local_polar
from module_kinematics import ssa


# === 1) ALLOCATION MATRIX (same as PID_TWIN, but functional) ===
def allocate_forces_to_thrusters(force_local_x, moment_local_z, lever=0.20):
    """
    Allocate surge force Fx and yaw moment Mz to port & starboard thrusters.

    TA = [ [ 1    1 ],
           [ -L   L ] ]

    Returns:
        T_port, T_stbd    (in same thrust units as used in your PWM fit)
    """
    TA = np.array([[1.0, 1.0],
                   [-lever, lever]])

    F = np.array([[force_local_x],
                  [moment_local_z]])

    T = np.linalg.pinv(TA) @ F
    return T.ravel()  # [T_port, T_stbd]


# === 2) Experimental thrust→PWM→actuator mapping (copied from PID_TWIN) ===
def pwm_to_act(pwm):
    """
    Clamp PWM to [1400,1600] and map to actuator command in [-1,1].
    """
    if pwm > 1600.0:
        pwm_truncated = 1600.0
    elif pwm < 1400.0:
        pwm_truncated = 1400.0
    else:
        pwm_truncated = pwm

    act = (pwm_truncated - 1500.0) / 100.0
    return act


def force_to_pwm_to_act(thrust):
    """
    Map thrust (negative=astern, positive=ahead) to actuator command [-1,1]
    using your experimental 2nd-order polynomial fits.
    """
    if thrust < -0.05:
        # Astern polynomial
        pwm = 8.394 * thrust**2 + 118.290 * thrust + 1544.198
        act_cmd = pwm_to_act(pwm)
        return act_cmd
    elif thrust > 0.05:
        # Ahead polynomial
        pwm = -5.419 * thrust**2 + 93.808 * thrust + 1455.677
        act_cmd = pwm_to_act(pwm)
        return act_cmd
    else:
        # Deadband around zero
        return 0.0


# === 3) NEW velocity_to_controls: velocity → heading → PID → [port, stbd] ===
def velocity_to_controls1(
    velocity,
    pursuer_state,
    evader_state,
    lever=0.20,
    F_const=5.0,
):
    """
    Convert a desired planar velocity (vx, vy) into
    PORT and STBD thruster actuator commands, using:

      1. desired_heading = atan2(vy, vx)
      2. heading_error = ssa(desired_heading - current_heading)
      3. PD on heading_error -> yaw moment Mz
      4. constant surge force F_const -> Fx
      5. thrust allocation matrix -> [T_port, T_stbd]
      6. thrust -> PWM -> actuator command using experimental fits

    Inputs:
        velocity      : (vx, vy)
        pursuer_state : numpy array, with heading at index 11, yaw rate at 5
        evader_state  : only used for optional diagnostics (compute_local_polar)

    Returns:
        port_act, stbd_act   (actuator commands in [-1,1])
    """

    vx, vy = velocity

    # -------------------------
    # 1) Desired heading from velocity
    # -------------------------
    pursuer_pos = np.asarray(pursuer_state[6:8], dtype=float)
    evader_pos  = np.asarray(evader_state[6:8], dtype=float)
    _, alpha_local = compute_local_polar(pursuer_pos, evader_pos)  # optional debug

    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        # No translational command -> keep current heading
        desired_heading = float(pursuer_state[11])
    else:
        desired_heading = float(np.arctan2(vy, vx))

    # -------------------------
    # 2) Heading error (psi error), wrapped to [-pi, pi]
    # -------------------------
    current_heading = float(pursuer_state[11])   # ψ
    heading_error   = ssa(desired_heading - current_heading)

    # -------------------------
    # 3) Yaw PD -> yaw moment Mz
    #     (PID concept, but here PD: P on heading err, D using yaw rate)
    # -------------------------
    Kp_psi = 0.6
    Kd_psi = 0.05

    yaw_rate = float(pursuer_state[5])
    Mz = Kp_psi * heading_error - Kd_psi * yaw_rate

    # -------------------------
    # 4) Constant surge force (same speed all the time)
    # -------------------------
    Fx = float(F_const)

    # -------------------------
    # 5) Allocate Fx, Mz -> port & stbd thrusts
    # -------------------------
    T_port, T_stbd = allocate_forces_to_thrusters(Fx, Mz, lever=lever)

    # Optional: clamp raw thrust a bit to keep within calibrated region
    T_port = float(np.clip(T_port, -10.0, 10.0))
    T_stbd = float(np.clip(T_stbd, -10.0, 10.0))

    # -------------------------
    # 6) Convert thrust -> PWM -> actuator command using experimental fits
    #     (this is exactly the same logic as in PID_TWIN.publish_thrust)
    # -------------------------
    port_act = force_to_pwm_to_act(T_port)
    stbd_act = force_to_pwm_to_act(T_stbd)

    # -------------------------
    # Debug print
    # -------------------------
    print(f"[VEL→THR] pos={pursuer_pos}, "
          f"des_hdg_deg={np.degrees(desired_heading):.1f}, "
          f"cur_hdg_deg={np.degrees(current_heading):.1f}, "
          f"hdg_err_deg={np.degrees(heading_error):.1f}, "
          f"Fx={Fx:.2f}, Mz={Mz:.2f}, "
          f"T_port={T_port:.2f}, T_stbd={T_stbd:.2f}, "
          f"act_port={port_act:.2f}, act_stbd={stbd_act:.2f}")

    return port_act, stbd_act
