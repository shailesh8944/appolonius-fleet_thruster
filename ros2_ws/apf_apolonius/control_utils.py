
import numpy as np
from cooperative_pursuer import compute_local_polar
from module_kinematics import ssa



def velocity_to_controls(velocity, pursuer_state, evader_state):
    vx, vy = velocity

    pursuer_pos = np.asarray(pursuer_state[6:8], dtype=float)
    evader_pos = np.asarray(evader_state[6:8], dtype=float)
    _, alpha = compute_local_polar(pursuer_pos, evader_pos)

    if abs(vx) < 1e-6 and abs(vy) < 1e-6:
        desired_heading = float(pursuer_state[11])
    else:
        desired_heading = np.arctan2(vy, vx)

    current_heading = float(pursuer_state[11])
    heading_error = ssa(desired_heading - current_heading)
    speed = float(np.hypot(vx, vy))

    Kp = 8
    Kd = 0.8
    yaw_rate = float(pursuer_state[5])
    raw_rudder_command = Kp * heading_error + Kd * (-yaw_rate)

    deadband_deg = 1.0
    if abs(heading_error) < np.radians(deadband_deg):
        raw_rudder_command = 0.0

    try:
        prev_rudder = float(pursuer_state[12])
    except Exception:
        prev_rudder = 0.0

    dt = 0.1
    tau = 0.1
    alpha_lp = dt / (tau + dt)
    filtered = (1.0 - alpha_lp) * prev_rudder + alpha_lp * raw_rudder_command

    max_rate_deg_s = 18.0
    max_delta = np.radians(max_rate_deg_s) * dt
    delta = np.clip(filtered - prev_rudder, -max_delta, max_delta)
    rudder_rad = prev_rudder + delta
    max_rudder = np.radians(35.0)
    rudder_rad = float(np.clip(rudder_rad, -max_rudder, max_rudder))

    try:
        pursuer_state[12] = rudder_rad
    except Exception:
        pass

    rudder_cmd_deg = np.degrees(rudder_rad)
    nominal_speed = 0.5
    base_rpm = 5.0
    rpm = base_rpm * (speed / nominal_speed)
    rpm = float(np.clip(rpm, 1.0, 5.0))

    print(f"[CTRL] pos={pursuer_pos}, alpha_deg={np.degrees(alpha):.1f}, "
          f"cur_deg={np.degrees(current_heading):.1f}, des_deg={np.degrees(desired_heading):.1f}, "
          f"rudder_deg={rudder_cmd_deg:.1f}, rpm={rpm:.2f}")

    return rpm, rudder_cmd_deg
