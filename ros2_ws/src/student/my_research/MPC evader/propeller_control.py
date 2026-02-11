import numpy as np

def generate_pursuit_propeller(pursuer_state, evader_state):
    """
    Generate propeller RPM command for pursuit with improved dynamics
    
    Args:
        pursuer_state: State vector [u,v,r,x,y,psi,delta,n]
        evader_state: State vector [u,v,r,x,y,psi,delta,n]
    
    Returns:
        float: Propeller RPM command (0 to 800)
    """
    # Extract positions and velocities
    dx = evader_state[3] - pursuer_state[3]  # x distance
    dy = evader_state[4] - pursuer_state[4]  # y distance
    distance = np.sqrt(dx**2 + dy**2)
    
    # Get velocities
    pursuer_speed = np.sqrt(pursuer_state[0]**2 + pursuer_state[1]**2)
    evader_speed = np.sqrt(evader_state[0]**2 + evader_state[1]**2)
    
    # Calculate desired heading
    desired_heading = np.arctan2(dy, dx)
    heading_error = np.abs(np.arctan2(np.sin(desired_heading - pursuer_state[5]), 
                                     np.cos(desired_heading - pursuer_state[5])))
    
    # Base RPM settings
    min_rpm = 0       # Minimum RPM for maintaining way
    max_rpm = 100       # Maximum RPM
    cruise_rpm = 40    # Normal cruising RPM
    
    # Dynamic RPM control based on distance and heading
    if distance > 200:
        # Long range pursuit - maximum speed
        rpm = max_rpm
    elif distance < 80:
        # Close range - match evader speed
        if pursuer_speed > evader_speed:
            rpm = min_rpm
        else:
            rpm = cruise_rpm
    else:
        # Medium range - balance distance and heading
        distance_factor = (distance - 10) / 250  # 0 to 1
        heading_factor = 1 - (heading_error / np.pi)  # 1 when aligned, 0 when opposite
        
        # Combine factors with weights
        rpm = min_rpm + (max_rpm - min_rpm) * (0.7 * distance_factor + 0.3 * heading_factor)
    
    # Add acceleration limiting
    current_rpm = pursuer_state[7]
    max_rpm_change = 50  # Maximum RPM change per step
    rpm = current_rpm + np.clip(rpm - current_rpm, -max_rpm_change, max_rpm_change)
    
    # Debug output
    print(f"\nPursuer Propeller Debug:")
    print(f"Distance: {distance:.1f}")
    print(f"Speeds - Pursuer: {pursuer_speed:.2f}, Evader: {evader_speed:.2f}")
    print(f"Current/Target RPM: {current_rpm:.1f}/{rpm:.1f}")
    
    return float(np.clip(rpm, min_rpm, max_rpm))

def generate_evasion_propeller(evader_state, pursuer_states, goal):
    """
    Generate propeller RPM command for evasion
    
    Args:
        evader_state: State vector [u,v,r,x,y,psi,delta,n]
        pursuer_states: List of pursuer state vectors
        goal: Goal position [x, y]
    
    Returns:
        float: Propeller RPM command (0 to 800)
    """
    # Calculate distances to pursuers
    pursuer_distances = []
    for pursuer_state in pursuer_states:
        dx = pursuer_state[3] - evader_state[3]
        dy = pursuer_state[4] - evader_state[4]
        distance = np.sqrt(dx**2 + dy**2)
        pursuer_distances.append(distance)
    
    min_distance = min(pursuer_distances)
    
    # Calculate goal parameters
    dx_goal = goal[0] - evader_state[3]
    dy_goal = goal[1] - evader_state[4]
    goal_distance = np.sqrt(dx_goal**2 + dy_goal**2)
    
    # Base RPM settings
    min_rpm = 0.1
    max_rpm = 1.0
    cruise_rpm =0.5
    
    # RPM strategy based on situation
    if min_distance < 0.5:
        # Emergency evasion - maximum speed
        rpm = max_rpm
    elif min_distance < 2:
        # Active evasion - scale with distance
        distance_factor = (2 - min_distance) / 10  # 1 to 0
        rpm = cruise_rpm + (max_rpm - cruise_rpm) * distance_factor
    elif goal_distance > 10:
        # Goal seeking - cruise speed
        rpm = cruise_rpm
    else:
        # Goal approach - reduce speed
        rpm = min_rpm + (cruise_rpm - min_rpm) * (goal_distance / 10)
    
    # Add acceleration limiting
    current_rpm = evader_state[7]
    max_rpm_change = 0.1  # Maximum RPM change per step
    rpm = current_rpm + np.clip(rpm - current_rpm, -max_rpm_change, max_rpm_change)
    
    # Debug output
    print(f"\nEvader Propeller Debug:")
    print(f"Nearest pursuer: {min_distance:.1f}")
    print(f"Goal distance: {goal_distance:.1f}")
    print(f"Current/Target RPM: {current_rpm:.1f}/{rpm:.1f}")
    
    return float(np.clip(rpm, min_rpm, max_rpm))