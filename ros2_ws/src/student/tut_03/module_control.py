import numpy as np

"""
Rudder control module for vessel simulation.

This module provides different rudder control strategies for vessel steering:
- Fixed angle control
- Switching (alternating) angle control
"""

def fixed_rudder(t, state, rudder_angle=5.0):
    """Return a constant rudder angle command.
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        
    Returns:
        float: Commanded rudder angle in radians
        
    Example:
        >>> fixed_rudder(10.0, state_vector)
        0.0873  # 5.0 degrees in radians
    """
    delta_c = 0.0
    #===========================================================================
    # TODO: Implement the fixed rudder control
    #===========================================================================
    # Write your code here

    delta_c = np.radians(rudder_angle)
    

    #===========================================================================
    return delta_c

def switching_rudder(t, state):
    """Return an alternating rudder angle command that switches every 10 seconds.
    
    Args:
        t (float): Current simulation time [s]
        state (ndarray): Current vessel state vector
        
    Returns:
        float: Commanded rudder angle in radians, alternating between +/-5.0 degrees
        
    Example:
        >>> switching_rudder(5.0, state_vector)
        0.0873  # +5.0 degrees in radians
        >>> switching_rudder(15.0, state_vector)
        -0.0873  # -5.0 degrees in radians
    """
    delta_c = 0.0
    #===========================================================================
    # TODO: Implement the switching rudder control
    #===========================================================================
    # Write your code here
    rudder_angle = 5.0
    if (t//10) % 2 == 0:
        delta_c = np.radians(rudder_angle)
    
    else:
        delta_c = -np.radians(rudder_angle)



    #===========================================================================
    return delta_c
