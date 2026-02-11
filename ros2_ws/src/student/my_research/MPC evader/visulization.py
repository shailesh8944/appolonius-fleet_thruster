import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import platform
import matplotlib
class PursuitAnimation:

    def __init__(self, states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
                 time, obs_pos, obst_r, goal_evader, X0_pursuer1, X0_pursuer2, X0_pursuer3, X0_evader):
        """Initialize animation with simulation data"""
        self.l = 23/2  # Characteristic length for non-dimensionalization
        
        # Copy and non-dimensionalize states (only positions at indices 3 and 4)
        self.states_pursuer1 = states_pursuer1.copy()
        self.states_pursuer2 = states_pursuer2.copy()
        self.states_pursuer3 = states_pursuer3.copy()
        self.states_evader = states_evader.copy()
        
        for states in [self.states_pursuer1, self.states_pursuer2, self.states_pursuer3, self.states_evader]:
            states[:, 3] = states[:, 3] / self.l  # x position
            states[:, 4] = states[:, 4] / self.l  # y position

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
            X0[3] = X0[3] / self.l  # x position
            X0[4] = X0[4] / self.l  # y position

        # Create figure in full screen without toolbar
        plt.rcParams['toolbar'] = 'None'
        self.fig = plt.figure(figsize=(10, 5))  # Single plot for animation
        
        self.ax = self.fig.add_subplot(111)
        
        # Make window full screen
        # mng = plt.get_current_fig_manager()
        # mng.window.state('zoomed')  # For Windows
        mng = plt.get_current_fig_manager()
        if matplotlib.get_backend() != 'TkAgg':
            try:
                if platform.system() == "Windows":
                    mng.window.state('zoomed')
                else:
                    mng.window.attributes('-zoomed', True)
            except AttributeError as e:
                print(f"Window zoom not supported: {e}")
        else:
            print("GUI features are not available in headless mode.")
        
        self.fig.tight_layout(pad=3)
        self.fig.patch.set_facecolor('white')
        self.ax.set_facecolor('whitesmoke')
        
        # Remove navigation buttons
        self.fig.canvas.toolbar_visible = False
        self.fig.canvas.header_visible = False
        self.fig.canvas.footer_visible = False

    def draw_ship(self, ax, x, y, psi, color='b'):
        """Draw a simplified ship shape at the given position and heading"""
        length = 1.8/self.l;  # Ship length
        width = 0.4/self.l  # Ship width
        
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
                           facecolor=color, alpha=0.8)
    def draw_capture_circle(self, ax, evader_pos, radius=15/230, color='red', alpha=0.2):
     """Draw a capture circle around the evader"""
     circle = plt.Circle((evader_pos[0], evader_pos[1]), 
                       radius, 
                       color=color, 
                       alpha=alpha,
                       fill=True,
                       linestyle='--')  
     return ax.add_patch(circle)

    def live_update(self, states_p1, states_p2, states_p3, states_e, current_time):
        """Real-time update of the animation"""
        self.ax.clear()
        
        # Copy and non-dimensionalize input states
        state_arrays = [states_p1.copy(), states_p2.copy(), states_p3.copy(), states_e.copy()]
        for state in state_arrays:
            state[:, 3] = state[:, 3] /self.l
            state[:, 4] = state[:, 4] / self.l

        # Unpack non-dimensionalized states
        s_p1, s_p2, s_p3, s_e = state_arrays
        evader_pos = s_e[-1, 3:5] # Get current evader position
        self.draw_capture_circle(self.ax, evader_pos)

        # Set plot limits with padding
        padding = 0.5
        x_min = min(np.min(s[:, 3]) for s in state_arrays) - padding
        x_max = max(np.max(s[:, 3]) for s in state_arrays) + padding
        y_min = min(np.min(s[:, 4]) for s in state_arrays) - padding
        y_max = max(np.max(s[:, 4]) for s in state_arrays) + padding
        
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        
        # Plot trajectories with proper labels and colors for each pursuer
        pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
        pursuer_colors = ['blue', 'green', 'magenta']
        pursuer_states = [s_p1, s_p2, s_p3]

        for state, color, label in zip(pursuer_states, pursuer_colors, pursuer_labels):
            self.ax.plot(state[:, 3], state[:, 4], color=color, lw=2, alpha=0.8, label=label)
        
        # Plot evader trajectory
        self.ax.plot(s_e[:, 3], s_e[:, 4], 'r-', lw=2, alpha=0.7, label='Evader')
        
        # Draw ships for each pursuer and the evader at the current time (last point)
        for state, color in zip(pursuer_states, pursuer_colors):
            ship = self.draw_ship(self.ax, state[-1, 3], state[-1, 4], state[-1, 5], color)
            self.ax.add_patch(ship)
        evader_ship = self.draw_ship(self.ax, s_e[-1, 3], s_e[-1, 4], s_e[-1, 5], 'red')
        self.ax.add_patch(evader_ship)
        
        # Plot obstacles
        for i in range(len(self.obs_pos[0])):
            circle = plt.Circle((self.obs_pos[0][i], self.obs_pos[1][i]),
                                self.obst_r[i], color='gray', alpha=0.3)
            self.ax.add_patch(circle)
        
        # Plot goal for evader
        self.ax.plot(self.goal_evader[0], self.goal_evader[1], 'k*',
                     markersize=20, label='Goal')
        
        self.ax.grid(True, linestyle='--', alpha=0.6)
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
        self.ax.set_xlim([-50, 100/ self.l])
        self.ax.set_ylim([-50, 100/ self.l])
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
            ship = self.draw_ship(self.ax, state[3], state[4], state[5], color)
            ship.set_label(label)
            ship.set_alpha(0)  # Initially invisible
            self.ax.add_patch(ship)
            self.ship_patches.append(ship)
        
        return self.ship_patches + self.trajectory_lines + [self.trajectory_evader]

    def update_animation(self, frame):
        """Update animation frame"""
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_xlim([-50, 100/ self.l])
        self.ax.set_ylim([-50, 100/ self.l])
        self.ax.set_xlabel("X/L Position")
        self.ax.set_ylabel("Y/L Position")
        
        # Redraw obstacles and goal
        for i in range(len(self.obst_r)):
            obstacle = plt.Circle((self.obs_pos[0][i], self.obs_pos[1][i]),
                                  self.obst_r[i], color='red', alpha=0.3)
            self.ax.add_patch(obstacle)
        self.ax.scatter(self.goal_evader[0], self.goal_evader[1],
                        color='green', marker='*', s=100, zorder=5, label='Evader Goal')
        
        # Plot trajectories up to current frame
        pursuer_states = [self.states_pursuer1, self.states_pursuer2, self.states_pursuer3]
        pursuer_colors = ['b', 'g', 'm']
        pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
        for state, color, label in zip(pursuer_states, pursuer_colors, pursuer_labels):
            self.ax.plot(state[:frame+1, 3], state[:frame+1, 4],
                         color=color, lw=2, alpha=0.7, label=label)
        self.ax.plot(self.states_evader[:frame+1, 3], self.states_evader[:frame+1, 4],
                     'r-', lw=2, alpha=0.5, label='Evader')
        
        # Draw current ship positions
        ship_states = pursuer_states + [self.states_evader]
        ship_colors = pursuer_colors + ['red']
        for state, color in zip(ship_states, ship_colors):
            if frame < len(state):
                x, y, psi = state[frame, 3], state[frame, 4], state[frame, 5]
                ship = self.draw_ship(self.ax, x, y, psi, color)
                self.ax.add_patch(ship) 
            if frame < len(self.states_evader):
                evader_pos = self.states_evader[frame, 3:5]
                self.draw_capture_circle(self.ax, evader_pos)     
        
        self.ax.legend()
        self.ax.set_title(f'Time: {self.time[frame]:.1f}s' if frame < len(self.time) else 'Completed')
        
        return []

    def save_frames(self, save_dir):
        """Save individual frames at approximately 1-second intervals"""
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            print(f"Created directory: {save_dir}")

        # Calculate frame interval for approximately 1-second intervals
        frame_interval = int(1.0 / (self.time[1] - self.time[0]))
        
        for frame in range(0, len(self.states_pursuer1), frame_interval):
            self.ax.clear()
            self.ax.grid(True)
            self.ax.set_xlim([-50, 100/self.l])
            self.ax.set_ylim([-50, 100/ self.l])
            self.ax.set_xlabel("X/L ")
            self.ax.set_ylabel("Y/L ")
            
            # Redraw obstacles and goal
            for i in range(len(self.obst_r)):
                obstacle = plt.Circle((self.obs_pos[0][i], self.obs_pos[1][i]),
                                    self.obst_r[i], color='red', alpha=0.3)
                self.ax.add_patch(obstacle)
            self.ax.scatter(self.goal_evader[0], self.goal_evader[1],
                            color='black', marker='*', s=200, zorder=5, label='Evader Goal')
            
            # Plot trajectories up to current frame
            pursuer_states = [self.states_pursuer1, self.states_pursuer2, self.states_pursuer3]
            pursuer_colors = ['b', 'g', 'm']
            pursuer_labels = ['Pursuer 1', 'Pursuer 2', 'Pursuer 3']
            for state, color, label in zip(pursuer_states, pursuer_colors, pursuer_labels):
                self.ax.plot(state[:frame+1, 3], state[:frame+1, 4],
                            color=color, lw=2, alpha=0.8, label=label)
            self.ax.plot(self.states_evader[:frame+1, 3], self.states_evader[:frame+1, 4],
                        'r-', lw=2, alpha=0.6, label='Evader')
            
            # Draw current ship positions
            ship_states = pursuer_states + [self.states_evader]
            ship_colors = pursuer_colors + ['red']
            for state, color in zip(ship_states, ship_colors):
                if frame < len(state):
                    x, y, psi = state[frame, 3], state[frame, 4], state[frame, 5]
                    ship = self.draw_ship(self.ax, x, y, psi, color)
                    self.ax.add_patch(ship)
                if frame < len(self.states_evader):
                    evader_pos = self.states_evader[frame, 3:5]
                    self.draw_capture_circle(self.ax, evader_pos)     
            
            self.ax.legend()
            self.ax.set_title(f'Time: {self.time[frame]:.1f}s' if frame < len(self.time) else 'Completed')
            
            # Save frame
            frame_path = os.path.join(save_dir, f'frame_{frame:03d}.pdf')
            plt.savefig(frame_path, dpi=1000, bbox_inches='tight',facecolor='none',edgecolor='none')
            print(f"Saved frame {frame} to {frame_path}")

    def create_animation(self, save_path):
        """Create and save the animation"""
        # Save individual frames first
        frames_dir = os.path.join(os.path.dirname(save_path), 'frames')
        self.save_frames(frames_dir)
        
        # Create and save the animation
        ani = FuncAnimation(self.fig, self.update_animation,
                            frames=len(self.states_pursuer1),
                            init_func=self.init_animation,
                            blit=False, repeat=False)
        
        print(f"Saving animation to {save_path}...")
        ani.save(save_path, writer='pillow', fps=10)
        print("Animation saved successfully!")