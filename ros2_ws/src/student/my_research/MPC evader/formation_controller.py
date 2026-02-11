import casadi as ca
from scipy.optimize import linear_sum_assignment

class FormationController:
    def __init__(self, formation_radius=20):
        self.formation_radius = formation_radius

    def calculate_formation_positions(self, evader_state, pursuers_states):
        """Calculate desired formation positions around the evader"""
        xe, ye = evader_state[0, 1], evader_state[0, 2]
        evader_heading = evader_state[0, 2]

        # Define angles for triangular formation
        angles = [0, 2 * ca.pi / 3, 4 * ca.pi / 3]
        formation_positions = []
        
        for angle in angles:
            pos_x = xe + self.formation_radius * ca.cos(evader_heading + angle)
            pos_y = ye + self.formation_radius * ca.sin(evader_heading + angle)
            formation_positions.append((pos_x, pos_y))
        
        return formation_positions
    
    def assign_positions(self, formation_positions, pursuer_states):
        """Assign pursuers to formation positions using Hungarian algorithm"""
        try:
            cost_matrix = [[ca.norm_2(ca.vertcat(state[3] - target[0], state[4] - target[1]))
                            for target in formation_positions] 
                           for state in pursuer_states]
            
            cost_matrix_np = ca.evalf(cost_matrix)  # Convert CasADi expressions to NumPy for assignment
            row_ind, col_ind = linear_sum_assignment(cost_matrix_np)
            
            return {i: formation_positions[j] for i, j in zip(row_ind, col_ind)}
        
        except Exception as e:
            print(f"Error in assign_positions: {e}")
            return {i: (0, 0) for i in range(len(pursuer_states))}

    def calculate_formation_control(self, pursuer_state, target_position):
        """
        CasADi-compatible version: Calculate control input for maintaining formation.
        Returns: CasADi function for control inputs [rpm_command, rudder_command]
        """
        # Define CasADi symbolic variables
        xp, yp, current_heading = ca.SX.sym('xp'), ca.SX.sym('yp'), ca.SX.sym('current_heading')
        u, v = ca.SX.sym('u'), ca.SX.sym('v')  # Surge and sway velocity
        n_rps = ca.SX.sym('n_rps')  # Current RPM
        speed_error_integral = ca.SX.sym('speed_error_integral')

        current_speed = ca.sqrt(u**2 + v**2)
        dx, dy = target_position[0] - xp, target_position[1] - yp
        desired_heading = ca.atan2(dy, dx)
        distance = ca.sqrt(dx**2 + dy**2)
        heading_error = ca.atan2(ca.sin(desired_heading - current_heading), ca.cos(desired_heading - current_heading))

        # Propeller characteristics
        Dp, wp = 7.9, 1 - 0.645
        desired_speed = 1.4  
        Va_desired = desired_speed * (1 - wp)
        J_optimal = 0.7
        n_required=Va_desired / (J_optimal * Dp)
        rps_required = n_required 

        # Current advance coefficient
        current_Va = current_speed * (1 - wp)
        current_n = ca.fmax(n_rps, 0.001)
        J_current = current_Va / (current_n * Dp)

        # Thrust coefficient polynomial model
        a0, a1, a2 = 0.5228, -0.4390, -0.0609
        Kt = a0 + a1 * J_current + a2 * J_current**2

        # RPM PID control
        Kp_prop, Ki_prop = 1.5, 0.5
        speed_error = desired_speed - current_speed
        speed_error_integral_new = speed_error_integral + speed_error * 0.1
        
        rps_command = rps_required + Kp_prop * speed_error + Ki_prop * speed_error_integral_new
        rps_command = ca.fmax(ca.fmin(rps_command, 1.5), 1.0)

        # Rudder control
        Kp_rudder, Kd_rudder = 1.6, 0.8
        rudder_command = Kp_rudder * heading_error * (1 + Kd_rudder * distance/self.formation_radius)
        rudder_command = ca.fmax(ca.fmin(rudder_command * 180/ca.pi, 35.0), -35.0)

        # Convert to CasADi function
        control_function = ca.vertcat(rps_command, rudder_command)

        
        return control_function
