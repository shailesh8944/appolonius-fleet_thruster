"""
File: plot_results.py
Description: This file provides visualization utilities for the MAV simulator results.
             It includes functions for:
             
             - Creating and initializing live plots during simulation
             - Updating plots with real-time simulation data
             - Generating comprehensive post-simulation visualizations
             - Animating vessel trajectories and state evolution
             - Displaying multiple vessel states and parameters simultaneously
             
             This module enhances the simulator with visual feedback capabilities,
             making it easier to interpret simulation results and verify vessel behavior.
             It supports both real-time visualization during simulation and
             post-processing of recorded simulation data.
             
Author: MAV Simulator Team
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def init_live_plot():
    """Initialize the live plotting figure and axes.
    
    Returns:
        tuple: (figure, axes, line objects) for live updating
    """
    fig = plt.figure(figsize=(10, 8))
    
    # Position plot (top)
    ax1 = fig.add_subplot(211)
    line1, = ax1.plot([], [], 'b-', label='Path')
    point1, = ax1.plot([], [], 'ro', label='Current Position')
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title('AUV Position (Live)')
    ax1.grid(True)
    ax1.legend()
    
    # Velocity plot (bottom)
    ax2 = fig.add_subplot(212)
    line2, = ax2.plot([], [], 'g-', label='Speed')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Speed [m/s]')
    ax2.set_title('AUV Speed')
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    
    return fig, (ax1, ax2), (line1, point1, line2)

def update_live_plot(frame_data, axes, lines):
    """Update the live plot with new data.
    
    Args:
        frame_data: tuple of (time, position, velocity) data
        axes: tuple of plot axes
        lines: tuple of line objects to update
    """
    time, pos, vel = frame_data
    ax1, ax2 = axes
    line1, point1, line2 = lines
    
    # Update position plot
    line1.set_data(pos[0], pos[1])
    point1.set_data([pos[0][-1]], [pos[1][-1]])
    
    # Update velocity plot
    line2.set_data(time, vel)
    
    # Adjust axes limits if needed
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

def plot_simulation_results(vessel):
    """Plot vessel simulation results.
    
    Args:
        vessel: Vessel object containing simulation history
    """
    # Extract time vector
    time = np.linspace(0, vessel.Tmax, vessel.history.shape[0])
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 10))
    
    # Plot 1: Vessel path
    ax1 = fig.add_subplot(321)
    ax1.plot(vessel.history[:, 6], vessel.history[:, 7])
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title('Vessel Path')
    ax1.grid(True)
    
    # Plot 2: Velocities
    ax2 = fig.add_subplot(322)
    ax2.plot(time, vessel.history[:, 0], label='u')
    ax2.plot(time, vessel.history[:, 1], label='v')
    ax2.plot(time, np.sqrt(vessel.history[:, 0]**2 + vessel.history[:, 1]**2), 
             label='Total Speed')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.set_title('Vessel Velocities')
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Turn rate
    ax3 = fig.add_subplot(323)
    ax3.plot(time, vessel.history[:, 5])
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Turn Rate [rad/s]')
    ax3.set_title('Turn Rate')
    ax3.grid(True)
    
    # Plot 4: Rudder angle
    ax4 = fig.add_subplot(324)
    ax4.plot(time, np.rad2deg(vessel.history[:, 12]))
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Rudder Angle [deg]')
    ax4.set_title('Rudder Angle')
    ax4.grid(True)
    
    # Plot 5: Heading angle
    ax5 = fig.add_subplot(325)
    ax5.plot(time, np.rad2deg(vessel.history[:, 11]))
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Heading Angle [deg]')
    ax5.set_title('Heading Angle')
    ax5.grid(True)
    
    plt.tight_layout()
    plt.show()
    plt.savefig('/workspaces/mavlab/simulation_results.png')