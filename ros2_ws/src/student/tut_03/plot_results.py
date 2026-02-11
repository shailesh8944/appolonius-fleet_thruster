import numpy as np
import matplotlib.pyplot as plt

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
    plt.savefig('/workspaces/mavlab/ros2_ws/src/student/tut_03/simulation_results.png')