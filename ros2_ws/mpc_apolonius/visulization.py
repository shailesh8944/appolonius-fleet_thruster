import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PursuitAnimation:

    def __init__(self, states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
                 time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2, X0_pursuer3, X0_evader,predicted_path=None):
        """Initialize animation with simulation data"""
        self.predicted_path=predicted_path
        self.l = 1  # Characteristic length for non-dimensionalization
        
        # Copy and non-dimensionalize states (only positions at indices 3 and 4)
        self.states_pursuer1 = states_pursuer1.copy()
        self.states_pursuer2 = states_pursuer2.copy()
        self.states_pursuer3 = states_pursuer3.copy()
        self.states_evader = states_evader.copy()
        
        for states in [self.states_pursuer1, self.states_pursuer2, self.states_pursuer3, self.states_evader]:
            states[:, 6] = states[:, 6] / self.l  # x position
            states[:, 7] = states[:, 7] / self.l  # y position

        self.time = time
        self.obs_pos = [np.array(obs_pos[0]) / self.l, np.array(obs_pos[1]) / self.l]
        self.obst_r = np.array(obst_r) / self.l
        self.goal_evader = np.array([goal_evader[0] / self.l, goal_evader[1] / self.l])
        
        # Non-dimensionalize initial positions
        self.X0_pursuer1 = X0_pursuer1.copy()
        self.X0_pursuer2 = X0_pursuer2.copy()
        self.X0_pursuer3 = X0_pursuer3.copy()
        self.X0_evader = X0_evader.copy()
        
        for X0 in [self.X0_pursuer1, self.X0_pursuer2, self.X0_pursuer3, self.X0_evader]:
            X0[6] = X0[6] / self.l  # x position
            X0[7] = X0[7] / self.l  # y position

        # Create figure in full screen without toolbar
        plt.rcParams['toolbar'] = 'None'
        self.fig = plt.figure(figsize=(10, 10))  # Single plot for animation
        
        self.ax = self.fig.add_subplot(111)
        
        # Make window full screen
        mng = plt.get_current_fig_manager()
        try:
            mng.window.state('zoomed')  # Works only for GUI backends (TkAgg, Qt5Agg)
        except AttributeError:
            pass  # Skip if running headless (no window)

      # For Windows  mng.window.state('zoomed')  # For Windows
        
        self.fig.tight_layout(pad=0)
        self.fig.patch.set_facecolor('white')
        self.ax.set_facecolor('whitesmoke')
        
        # Remove navigation buttons
        self.fig.canvas.toolbar_visible = False
        self.fig.canvas.header_visible = False
        self.fig.canvas.footer_visible = False

    def draw_ship(self, ax, x, y, psi, color='b'):
        """Draw a simplified ship shape at the given position and heading"""
        length = 3.0 / self.l  # Ship length
        width = 1.0 / self.l    # Ship width
        
        # Calculate ship corners
        bow_x = x + (length / 2) * np.cos(psi)
        bow_y = y + (length / 2) * np.sin(psi)
        stern_x = x - (length / 2) * np.cos(psi)
        stern_y = y - (length / 2) * np.sin(psi)
        port_x = x + (width / 2) * np.cos(psi + np.pi / 2)
        port_y = y + (width / 2) * np.sin(psi + np.pi / 2)
        starboard_x = x - (width / 2) * np.cos(psi + np.pi / 2)
        starboard_y = y - (width / 2) * np.sin(psi + np.pi / 2)
        
        ship_xs = [bow_x, port_x, stern_x, starboard_x]
        ship_ys = [bow_y, port_y, stern_y, starboard_y]
        
        return plt.Polygon(np.column_stack((ship_xs, ship_ys)),
                           facecolor=color, alpha=0.6)
    def draw_capture_circle(self, ax, evader_pos, radius=2, color='red', alpha=0.2):
     """Draw a capture circle around the evader"""
     circle = plt.Circle((evader_pos[0], evader_pos[1]), 
                       radius, 
                       color=color, 
                       alpha=alpha,
                       fill=True,
                       linestyle='--')  
     return ax.add_patch(circle)

    def live_update(self, states_p1, states_p2, states_p3, states_e, current_time,predicted_path=None):
        """Real-time update of the animation"""
        self.ax.clear()
        
        # Copy and non-dimensionalize input states
        state_arrays = [states_p1.copy(), states_p2.copy(), states_p3.copy(), states_e.copy()]
        for state in state_arrays:
            state[:, 6] = state[:, 6] / self.l
            state[:, 7] = state[:, 7] / self.l

        # Unpack non-dimensionalized states
        s_p1, s_p2, s_p3, s_e = state_arrays

        # Set plot limits with padding
        padding = 0.5
        x_min = min(np.min(s[:, 6]) for s in state_arrays) - padding
        x_max = max(np.max(s[:, 6]) for s in state_arrays) + padding
        y_min = min(np.min(s[:, 7]) for s in state_arrays) - padding
        y_max = max(np.max(s[:, 7]) for s in state_arrays) + padding
        
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        
        # Plot trajectories with proper labels and colors for each pursuer
        pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
        pursuer_colors = ['blue', 'green', 'magenta']
        pursuer_states = [s_p1, s_p2, s_p3]

        for state, color, label in zip(pursuer_states, pursuer_colors, pursuer_labels):
            self.ax.plot(state[:, 6], state[:, 7], color=color, lw=2, alpha=0.7, label=label)
        
        # Plot evader trajectory
        self.ax.plot(s_e[:, 6], s_e[:, 7], 'r-', lw=2, alpha=0.7, label='Evader')
        evader_pos = [s_e[-1, 6], s_e[-1, 7]]
        self.draw_capture_circle(self.ax, evader_pos)
        
        evader_ship = self.draw_ship(self.ax, s_e[-1, 6], s_e[-1, 7], s_e[-1, 11], 'red')
        self.ax.add_patch(evader_ship)
        if predicted_path is not None:
            # Non-dimensionalize predicted path
            pred_x = predicted_path[:, 0] / self.l
            pred_y = predicted_path[:, 1] / self.l
            self.ax.plot(pred_x, pred_y, 'b--', lw=2, alpha=0.7, label='Evader NMPC Prediction')

        # Draw ships for each pursuer and the evader at the current time (last point)
        for state, color in zip(pursuer_states, pursuer_colors):
            ship = self.draw_ship(self.ax, state[-1, 6], state[-1, 7], state[-1, 11], color)
            self.ax.add_patch(ship)
        evader_ship = self.draw_ship(self.ax, s_e[-1, 6], s_e[-1, 7], s_e[-1, 11], 'red')
        self.ax.add_patch(evader_ship)
        pursuer_states = [s_p1[-1], s_p2[-1], s_p3[-1]]
        evader_state=s_e[-1]
        # Plot obstacles
        for i in range(len(self.obs_pos[0])):
            circle = plt.Circle((self.obs_pos[0][i], self.obs_pos[1][i]),
                                self.obst_r[i], color='gray', alpha=0.3)
            self.ax.add_patch(circle)
        
        # Plot goal for evader
        self.ax.plot(self.goal_evader[0], self.goal_evader[1], 'k*',
                     markersize=20, label='Goal')
        for i, (pursuer_state, color) in enumerate(zip(pursuer_states, pursuer_colors)):
        # Calculate radius using speed ratio
            pursuer_speed = pursuer_state[0]  # Assuming speed is first element
            evader_speed = evader_state[0]
            
            if evader_speed != 0:
                lambda_i = pursuer_speed / evader_speed
                if lambda_i != 1:
                    # Calculate distance between pursuer and evader
                    dx = pursuer_state[6] - evader_state[6]
                    dy = pursuer_state[7] - evader_state[7]
                    R = np.sqrt(dx**2 + dy**2)
                    
                    # Calculate Apollonius circle radius
                    r_a = (R * lambda_i) / abs(1 - lambda_i**2) if lambda_i != 1 else R/2
                    
                    # Draw Apollonius circle
                    circle = plt.Circle(
                        (pursuer_state[3], pursuer_state[4]),
                        r_a,
                        color=color,
                        fill=False,
                        linestyle='--',
                        alpha=0.5,
                        label=f'P{i+1} Apollonius Circle'
                    )
                    self.ax.add_patch(circle)
        
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.set_xlabel('x/L', fontsize=12)
        self.ax.set_ylabel('y/L', fontsize=12)
        self.ax.set_title(f'Time: {current_time:.1f}s', fontsize=14, pad=10)
        self.ax.legend(loc='upper right', fontsize=10, framealpha=0.8)
        self.ax.set_aspect('equal')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def init_animation(self):
        """Initialize animation frame"""
        self.ax.grid(True)
        self.ax.set_xlim([-20, 60/ self.l])
        self.ax.set_ylim([-20, 60 / self.l])
        # Set up empty trajectory lines for all pursuers and the evader
        self.trajectory_lines = []
        pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
        pursuer_colors = ['b', 'g', 'm']
        for color, label in zip(pursuer_colors, pursuer_labels):
            line, = self.ax.plot([], [], color, linestyle='--', linewidth=0.5, label=f"{label} Path")
            self.trajectory_lines.append(line)
        self.trajectory_evader, = self.ax.plot([], [], 'r', linestyle='--', linewidth=0.5, label="Evader Path")
        
        # Add initial ships (set to invisible)
        self.ship_patches = []
        initial_states = [self.X0_pursuer1, self.X0_pursuer2, self.X0_pursuer3, self.X0_evader]
        colors = pursuer_colors + ['red']
        labels = pursuer_labels + ['Evader']
        for state, color, label in zip(initial_states, colors, labels):
            ship = self.draw_ship(self.ax, state[6], state[7], state[11], color)
            ship.set_label(label)
            ship.set_alpha(0)  # Initially invisible
            self.ax.add_patch(ship)
            self.ship_patches.append(ship)
        
        return self.ship_patches + self.trajectory_lines + [self.trajectory_evader]

    def update_animation(self, frame):
        """Update animation frame"""

        self.ax.clear()
        self.ax.grid(True)
        
        # Fix axis limits to be dynamic based on ship positions
        positions = [
            self.states_pursuer1[frame, 6:8],
            self.states_pursuer2[frame, 6:8],
            self.states_pursuer3[frame, 6:8],
            self.states_evader[frame, 6:8]
        ]
      
        self.ax.set_xlim([-20/self.l, 60/ self.l])
        self.ax.set_ylim([-20/self.l, 60/ self.l])
        
        self.ax.set_xlabel("X/L Position")
        self.ax.set_ylabel("Y/L Position")
        legend_elements = [
        plt.Line2D([0], [0], color='blue', lw=2, label='Pursuer 1 Path'),
        plt.Line2D([0], [0], color='green', lw=2, label='Pursuer 2 Path'),
        plt.Line2D([0], [0], color='magenta', lw=2, label='Pursuer 3 Path'),
        plt.Line2D([0], [0], color='red', lw=2, label='Evader Path'),
        plt.Rectangle((0,0), 1, 1, fc='blue', alpha=0.6, label='Pursuer 1'),
        plt.Rectangle((0,0), 1, 1, fc='green', alpha=0.6, label='Pursuer 2'),
        plt.Rectangle((0,0), 1, 1, fc='magenta', alpha=0.6, label='Pursuer 3'),
        plt.Rectangle((0,0), 1, 1, fc='red', alpha=0.6, label='Evader'),
        plt.Circle((0,0), 1, fc='gray', alpha=0.3, label='Obstacles'),
        plt.Circle((0,0), 1, fc='red', alpha=0.2, label='Capture Radius'),
        plt.Line2D([0], [0], marker='*', color='k', label='Goal',
                  markerfacecolor='k', markersize=15, linestyle='None'),
    ]
    
    # Add legend box with two columns
        self.ax.legend(handles=legend_elements, 
                    loc='center left',
                    bbox_to_anchor=(1.05, 0.5),
                    ncol=1,
                    fancybox=True,
                    shadow=True,
                    fontsize=10)
        
        # Adjust subplot parameters to give specified padding
        self.fig.subplots_adjust(right=0.85)
    
        
        # Draw obstacles and goal
        for i in range(len(self.obst_r)):
            obstacle = plt.Circle((self.obs_pos[0][i], self.obs_pos[1][i]),
                                self.obst_r[i], color='gray', alpha=0.3)
            self.ax.add_patch(obstacle)
            
        self.ax.plot(self.goal_evader[0], self.goal_evader[1], 'k*',
                     markersize=20, label='Goal')
        
        # Plot trajectories up to current frame
        pursuer_states = [self.states_pursuer1, self.states_pursuer2, self.states_pursuer3]
        pursuer_colors = ['blue', 'green', 'magenta']
        pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
        
        # Draw trajectories
        for state, color, label in zip(pursuer_states, pursuer_colors, pursuer_labels):
            self.ax.plot(state[:frame+1, 6], state[:frame+1, 7],
                        color=color, lw=2, alpha=0.7)
            
            # Draw current ship position
            if frame < len(state):
                ship = self.draw_ship(self.ax, state[frame, 6], state[frame, 7], 
                                    state[frame, 11], color)
                self.ax.add_patch(ship)
        
        # Draw evader trajectory and ship
        self.ax.plot(self.states_evader[:frame+1, 6], self.states_evader[:frame+1, 7],
                     'r-', lw=2, alpha=0.7)
        evader_pos = [self.states_evader[frame, 6], self.states_evader[frame, 7]]
        self.draw_capture_circle(self.ax, evader_pos)
        
        evader_ship = self.draw_ship(self.ax, 
                                    self.states_evader[frame, 6],
                                    self.states_evader[frame, 7],
                                    self.states_evader[frame, 11],
                                    'red')
        self.ax.add_patch(evader_ship)
        if self.predicted_path is not None and len(self.predicted_path) > 0:
            if frame < len(self.predicted_path):
                pred_path = self.predicted_path[frame]
            else:
                pred_path = self.predicted_path[-1]  # Use last available prediction for extra frames
            pred_x = pred_path[:, 0] / self.l
            pred_y = pred_path[:, 1] / self.l
            self.ax.plot(pred_x, pred_y, 'b--', lw=2, alpha=0.7, label='Evader NMPC Prediction')
        if frame < len(self.states_pursuer1):
            pursuer_states = [
                self.states_pursuer1[frame],
                self.states_pursuer2[frame],
                self.states_pursuer3[frame]
            ]
            evader_state = self.states_evader[frame]
            
            for i, (pursuer_state, color) in enumerate(zip(pursuer_states, pursuer_colors)):
                pursuer_speed = pursuer_state[0]
                evader_speed = evader_state[0]
                
                if evader_speed != 0:
                    lambda_i = pursuer_speed / evader_speed
                    if lambda_i != 1:
                        dx = pursuer_state[6] - evader_state[6]
                        dy = pursuer_state[7] - evader_state[7]
                        R = np.sqrt(dx**2 + dy**2)
                        
                        r_a = (R * lambda_i) / abs(1 - lambda_i**2) if lambda_i != 1 else R/2
                        r_a_=r_a/self.l
                        circle = plt.Circle(
                            (pursuer_state[6], pursuer_state[7]),
                            r_a_,
                            color=color,
                            fill=False,
                            linestyle='--',
                            alpha=0.5,
                            label=f'P{i+1} Apollonius Circle'
                        )
                        self.ax.add_patch(circle)
        

        legend = self.ax.legend(
        loc='upper right',       # Place legend inside top-right corner
        fontsize=10,
        framealpha=0.8,
        fancybox=True,
        shadow=True,
        facecolor='white'
    )
        legend.get_frame().set_edgecolor('gray')

    # ---- Add time text at top center ----
        self.ax.text(
        0.5, 1.02, f"Time: {self.time[frame]:.1f} s",
        transform=self.ax.transAxes,
        fontsize=13,
        fontweight='bold',
        color='black',
        ha='center', va='bottom',
        bbox=dict(facecolor='white', edgecolor='none', alpha=0.6, boxstyle='round,pad=0.3')
    )
        self.ax.legend(loc='upper right')
        # self.ax.set_title(f'Time: {self.time[frame]:.1f}s')
        self.ax.set_aspect('equal')
        
        return []

    
    def create_animation(self, save_path):
        """Create and save the animation"""
        # Create new figure with extra space for legend
        save_fig = plt.figure(figsize=(10, 10))  # Increased width for legend
        temp_ax = save_fig.add_subplot(111)
        
        # Store original axis
        orig_ax = self.ax
        self.ax = temp_ax
        
        # Add legend elements
        legend_elements = [
            plt.Line2D([0], [0], color='blue', lw=2, label='Pursuer 1 Path'),
            plt.Line2D([0], [0], color='green', lw=2, label='Pursuer 2 Path'),
            plt.Line2D([0], [0], color='magenta', lw=2, label='Pursuer 3 Path'),
            plt.Line2D([0], [0], color='red', lw=2, label='Evader Path'),
            plt.Rectangle((0,0), 1, 1, fc='blue', alpha=0.6, label='Pursuer 1'),
            plt.Rectangle((0,0), 1, 1, fc='green', alpha=0.6, label='Pursuer 2'),
            plt.Rectangle((0,0), 1, 1, fc='magenta', alpha=0.6, label='Pursuer 3'),
            plt.Rectangle((0,0), 1, 1, fc='red', alpha=0.6, label='Evader'),
            plt.Circle((0,0), 1, fc='gray', alpha=0.3, label='Obstacles'),
            plt.Circle((0,0), 1, fc='red', alpha=0.2, label='Capture Radius'),
            plt.Line2D([0], [0], marker='*', color='k', label='Goal',
                    markerfacecolor='k', markersize=15, linestyle='None'),
        ]
        
        def init():
            self.ax.clear()
            return []
        
        def update(frame):
            self.update_animation(frame)
            # Add legend in each frame
            self.ax.legend(handles=legend_elements,
                        loc='center left',
                        bbox_to_anchor=(0.02, 0.98),
                        ncol=1,
                        fancybox=True,
                        shadow=True,
                        fontsize=10)
            # Adjust layout to make room for legend
            self.fig.subplots_adjust(right=0.85)
            return []
        
        ani = FuncAnimation(save_fig, update,
                        frames=len(self.time)-1,
                        init_func=init,
                        blit=False,
                        repeat=False)
        
        print(f"Saving animation to {save_path}...")
        
        # Save with extra width to accommodate legend
        ani.save(save_path, 
                writer='pillow', 
                fps=10,
                dpi=200,
                )  # Add padding around the figure
        
        print("Animation saved successfully!")
        
        # Restore original axis and close temporary figure
        self.ax = orig_ax
        plt.close(save_fig)