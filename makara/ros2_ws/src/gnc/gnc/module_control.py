import numpy as np

"""
Rudder control module for vessel simulation.

This module provides different rudder control strategies for vessel steering:
"""

def ssa(ang, deg=False):
    """
    Smallest signed angle that lies between -pi and pi.
    If deg is True, the angle is assumed to be in degrees and is converted to radians.
    If deg is False, the angle is assumed to be in radians.

    Args:
        ang (float): Angle in degrees or radians
        deg (bool): If True, the angle is assumed to be in degrees

    Returns:
        float: Smallest signed angle between -pi and pi. The angle is in radians if deg is False, and in degrees if deg is True.
    """

    #===========================================================================
    # TODO: Implement the ssa function
    #===========================================================================
    # Write your code here

    if deg:
        ang = (ang + 180) % (360.0) - 180.0
    else:
        ang = (ang + np.pi) % (2 * np.pi) - np.pi

    #===========================================================================
    return ang

def clip(val, min_val, max_val):
    """
    Clip a value to a range.
    """
    return max(min_val, min(val, max_val))

def pid_control(t, state, waypoints, waypoint_idx, ye_int=0.0):
    """
    Implement a PID control strategy to follow the waypoints. 
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        waypoints (ndarray): Waypoints array
        waypoint_idx (int): Current waypoint index
        ye_int (float): Integral term of cross-track error
        
    Returns:
        float: Commanded rudder angle in radians
        int: Next waypoint index
    """

    # Check if we have reached the last waypoint
    if waypoint_idx == len(waypoints):
        return 0.0, 0.0, waypoint_idx

    # Current state
    u, v, r = state[3], state[4], state[5]
    x, y, psi = state[6], state[7], state[11]
    
    # Current and previous waypoints
    wp_xn, wp_yn, _ = waypoints[waypoint_idx]
    wp_xn1, wp_yn1, _ = waypoints[waypoint_idx - 1]

    delta_c = 0.0
    ye = 0.0
    #===========================================================================
    # TODO: Implement the PID control
    #===========================================================================
    # Write your code here
    
    a = -(wp_yn1 - wp_yn)
    b = (wp_xn1 - wp_xn)
    c = -wp_yn1*b-a*wp_xn1
    
    wp_unit_vec = np.array([wp_xn - wp_xn1, wp_yn - wp_yn1, 0.0])
    wp_unit_vec = wp_unit_vec / np.linalg.norm(wp_unit_vec)

    # Calculate cross-track error
    ye = -(a*x + b*y + c)/np.sqrt(a**2 + b**2)
    pi_p = np.angle(wp_unit_vec[0] + 1j * wp_unit_vec[1]) 


    Kpo = 0.6
    Kio = 0.05
    

    psid = pi_p + Kpo*(-ye) + Kio*(-ye_int)
   
    rd = 0

    Kpi = 0.7
    Kdi = 0.5

    delta_c =  Kpi*ssa(psid - psi) + Kdi*(rd - r)

    #===========================================================================
    
    # Clip the rudder angle
    delta_c = clip(delta_c, -35*np.pi/180, 35*np.pi/180)

    # Distance to waypoint
    wp_dist = np.linalg.norm(np.array([x - wp_xn, y - wp_yn, 0.0]))
    if wp_dist < 0.5:
        waypoint_idx += 1

    # Do not change the following line (the negative sign is due to the sign convention of the rudder angle)
    return -delta_c, ye, waypoint_idx