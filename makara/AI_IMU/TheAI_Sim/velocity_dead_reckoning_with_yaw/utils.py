import torch
import numpy as np

def body_to_global_velocity(vx_body, vy_body, yaw):
    """
    Converts velocities from the body frame to the global frame.

    Args:
        vx_body (torch.Tensor or float): Velocity along the x-axis in the body frame.
        vy_body (torch.Tensor or float): Velocity along the y-axis in the body frame.
        yaw (torch.Tensor or float): Yaw angle (radians).

    Returns:
        tuple: vx_global, vy_global
    """
    if isinstance(yaw, torch.Tensor):
        cos_yaw = torch.cos(yaw)
        sin_yaw = torch.sin(yaw)
    else:
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
    
    vx_global = vx_body * cos_yaw - vy_body * sin_yaw
    vy_global = vx_body * sin_yaw + vy_body * cos_yaw
    return vx_global, vy_global

def integrate_position(x_prev, y_prev, vx_global, vy_global, dt):
    """
    Integrates global velocities to update position.

    Args:
        x_prev (torch.Tensor or float): Previous x position in the global frame.
        y_prev (torch.Tensor or float): Previous y position in the global frame.
        vx_global (torch.Tensor or float): Velocity along the x-axis in the global frame.
        vy_global (torch.Tensor or float): Velocity along the y-axis in the global frame.
        dt (float): Time step.

    Returns:
        tuple: x_new, y_new
    """
    x_new = x_prev + vx_global * dt
    y_new = y_prev + vy_global * dt
    return x_new, y_new 