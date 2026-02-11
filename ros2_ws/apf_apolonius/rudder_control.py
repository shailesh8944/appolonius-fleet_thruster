import numpy as np


# def generate_evasion_rudder(current_state, pursuer_states, goal_state):
   
    
#     Kp = 1.2
#     Kd = 0.6

#     # Calculate center of pursuers
#     center_x = np.mean([p[3] for p in pursuer_states])
#     center_y = np.mean([p[4] for p in pursuer_states])
    
#     # Calculate threat direction
#     threat_x = current_state[3] - center_x
#     threat_y = current_state[4] - center_y
#     threat_dist = np.sqrt(threat_x**2 + threat_y**2)
    
#     # Calculate goal direction
#     goal_x = goal_state[0] - current_state[3]
#     goal_y = goal_state[1] - current_state[4]
#     goal_dist = np.sqrt(goal_x**2 + goal_y**2)
    
#     # Blend escape and goal directions
#     #threat_weight = np.exp(-threat_dist/100)
#     threat_weight = min(1.0, 0.8 * np.exp(-threat_dist / 300))  # Scale up threat weight
#     # desired_x = threat_weight * threat_x/threat_dist + (1-threat_weight) * goal_x/goal_dist
#     # desired_y = threat_weight * threat_y/threat_dist + (1-threat_weight) * goal_y/goal_dist
#     goal_weight = 1.0 - threat_weight  # Complementary weight for the goal direction
#     desired_x = threat_weight * threat_x / threat_dist + goal_weight * goal_x / goal_dist
#     desired_y = threat_weight * threat_y / threat_dist + goal_weight * goal_y / goal_dist
#     # Compute desired heading
#     desired_heading = np.arctan2(desired_y, desired_x)
    
#     # PD control
#     heading_error = np.arctan2(
#         np.sin(desired_heading - current_state[5]),
#         np.cos(desired_heading - current_state[5])
#     )
#     yaw_rate = current_state[2]
#     rudder_command = Kp * heading_error - Kd * yaw_rate
    
#     # Limit rudder
#     max_rudder = np.radians(35 )
#     rudder_command = np.clip(rudder_command, -max_rudder, max_rudder)
#     return rudder_command 
import numpy as np
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

# def generate_evasion_rudder(current_state, pursuer_states, goal_state):
#     """
#     APF-based evasion and goal-seeking rudder command.

#     current_state: [unused, unused, yaw_rate, x, y, heading]
#     pursuer_states: list of [unused, unused, unused, x, y, ...]
#     goal_state: [x, y]
#     """
#     # APF parameters
#     K_att = 100     # Attractive gain
#     K_rep = 20.0   # Repulsive gain
#     d0 = 600.0       # Influence distance for repulsion

#     # Attractive force (towards goal)
#     pos = np.array([current_state[3], current_state[4]])
#     goal = np.array(goal_state)
#     c=1e-2
#     F_att = K_att * (goal - pos+c)
    
#     # Repulsive force (from pursuers)
#     F_rep = np.zeros(2)
#     for pursuer in pursuer_states:
#         pursuer_pos = np.array([pursuer[3], pursuer[4]])
#         diff = pos - pursuer_pos
#         dist = np.linalg.norm(diff)
#         if dist < 1e-3:
#             continue  # Avoid division by zero
#         if dist < d0:
#             # Repulsive force magnitude
#             rep_mag = K_rep * (1.0/dist - 1.0/d0) / (dist**2)
#             F_rep += rep_mag * (diff / dist)
#         # else: No repulsion if outside influence distance

#     # Total force
#     F_total = F_att + F_rep

#     # Desired heading
#     desired_heading = np.arctan2(F_total[1], F_total[0])

#     # PD control for rudder
#     Kp = 1.2
#     Kd = 0.6
#     heading_error = ssa( current_state[5]-desired_heading)
#     yaw_rate = current_state[2]
#     rudder_command = Kp * heading_error - Kd * yaw_rate

#     # Limit rudder
#     max_rudder = np.radians(35)
#     rudder_command = np.clip(rudder_command, -max_rudder, max_rudder)
#     return rudder_command
# import numpy as np



# def generate_evasion_rudder(current_state, pursuer_states, goal_state):
#     """
#     APF-based evasion + goal-seeking rudder command, with PD and adaptive repulsion.
    
#     Inputs:
#       current_state: [_, _, yaw_rate, x, y, heading]
#       pursuer_states: list of [_, _, _, x, y, vx, vy]
#       goal_state: [x, y]
#     Returns:
#       rudder_command (float)
#     """

#     # --- 1. Hard-coded guidance & controller gains -------------
#     K_att       = 0.005        # attractive gain
#     K_rep_base  = 4000.0    # base repulsive gain
#     d0          = 500.0       # repulsion influence radius
#     eps         = 1e-6        # small epsilon
#     max_tc      = 30.0        # max time-to-collision for scaling

#     # PD gains
#     Kp          =1.5
#     Kd          = 0.
#     max_rudder  = np.radians(35)

#     # --- 2. Attractive force toward goal -----------------------
#     pos = np.array(current_state[3:5])
#     goal = np.array(goal_state)
#     F_att = K_att * (goal - pos)

#     # --- 3. Repulsive force from each pursuer ------------------
#     F_rep = np.zeros(2)
#     for p in pursuer_states:
#         p_pos = np.array(p[3:5])
#         p_vel = np.array(p[5:7])
#         diff  = pos - p_pos
#         dist  = np.linalg.norm(diff)
#         if dist < eps:
#             continue

#         # estimate closing speed and time-to-collision
#         closing_speed = np.dot(p_vel, -diff/dist)
#         if closing_speed > 0:
#             t_c = dist/closing_speed
#         else:
#             t_c = max_tc
#         t_c = min(t_c, max_tc)

#         # scale repulsion by how soon they collide
#         K_rep = K_rep_base / (1.0 + t_c)

#         if dist < d0:
#             rep_mag = K_rep * (1.0/dist - 1.0/d0) / (dist**2)
#             F_rep += rep_mag * (diff / dist)

#     # --- 4. Combine forces & compute desired heading ------------
#     F_total = F_att + F_rep
#     if np.linalg.norm(F_total) < eps:
#         psi_des = current_state[5]  # hold heading if no guidance vector
#     else:
#         psi_des = np.arctan2(F_total[1], F_total[0])

#     # --- 5. PD control on heading error ------------------------
#     heading  = current_state[5]
#     yaw_rate = current_state[2]
#     err = ssa(psi_des - heading)

#     rudder_cmd = Kp * err - Kd * yaw_rate

#     # --- 6. Saturate rudder ------------------------------------
#     return np.clip(rudder_cmd, -max_rudder, max_rudder)
def generate_evasion_rudder(current_state, pursuer_states, goal_state):
    """
    APF-based evasion + goal-seeking rudder command with enhanced evasion behavior
    """
    # --- 1. Gains tuned to avoid circling and keep goal pull ---
    K_att = 40           # stronger pull to goal
    K_rep_base = 800.0     # softer repulsion to reduce orbiting
    d0 = 10.0                # influence radius
    eps = 1e-6
    max_tc = 12.0

    Kp = 0.08
    Kd = 0.4
    max_rudder_deg = 35.0
    max_rudder = np.radians(max_rudder_deg)

    pos = np.array(current_state[6:8])
    goal = np.array(goal_state)

    closest_pursuer_dist = min(np.linalg.norm(np.array(p[6:8]) - pos) for p in pursuer_states)

    # Blend: when pursuers are close, keep some goal pull instead of shutting it down.
    danger = np.exp(-closest_pursuer_dist / (1.5 * d0))  # 1 when very close, ->0 when far
    attraction_weight = 0.35 + 0.65 * (1.0 - danger)    # never below 0.35
    F_att = K_att * attraction_weight * (goal - pos)

    F_rep = np.zeros(2)
    for p in pursuer_states:
        p_pos = np.array(p[6:8])
        p_vel = np.array(p[0:2])
        diff = pos - p_pos
        dist = np.linalg.norm(diff)
        if dist < eps:
            continue

        rel_vel = p_vel
        closing_speed = max(0.1, np.dot(rel_vel, -diff / dist))
        t_c = min(dist / closing_speed, max_tc)

        K_rep = K_rep_base * np.exp(-dist / (1.5 * d0)) / (1.0 + 0.5 * t_c)

        if dist < d0:
            rep_mag = K_rep * (1.0 / dist - 1.0 / d0) / (dist**1.5)
            F_rep += rep_mag * (diff / dist)

    F_total = F_att + F_rep

    if np.linalg.norm(F_total) < eps:
        closest_pursuer = min(pursuer_states, key=lambda p: np.linalg.norm(np.array(p[6:8]) - pos))
        diff_closest = pos - np.array(closest_pursuer[6:8])
        psi_des = np.arctan2(diff_closest[1], diff_closest[0])
    else:
        psi_des = np.arctan2(F_total[1], F_total[0])

    heading = current_state[11]
    yaw_rate = current_state[5]
    err = ssa(psi_des - heading)

    emergency_turn = 0.0
    if closest_pursuer_dist < d0 / 3:
        emergency_turn = np.sign(err) * 0.25 * max_rudder

    rudder_cmd = Kp * err - Kd * yaw_rate + emergency_turn
    rudder_cmd_deg = np.rad2deg(rudder_cmd)
    print("Evasion_____________________________ rudder command (deg):", rudder_cmd_deg)
    return float(np.clip(rudder_cmd_deg, -max_rudder_deg, max_rudder_deg))
