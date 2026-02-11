
import numpy as np
import os
from tqdm import tqdm
import matplotlib
matplotlib.use('Agg')  # headless-safe
import matplotlib.pyplot as plt
import casadi as cd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator  
from mpc_evader import controller
from kcs_ode import simulation
from module_kinematics import ssa,quat_to_eul
from visulization import PursuitAnimation
from plot_analysis import generate_analysis_plots
from cooperative_pursuer import (
    ApolloniusTradeoffController,
    compute_local_polar,
    compute_group_occupied_angle,
    compute_coverage_angles,
    sort_pursuers_by_angle
)
from control_utils import velocity_to_controls


# ===================== ROS 2 Bridge Node =====================
class VesselROSBridge(Node):
    """
    Subscribes to four Float64MultiArray vessel_state topics and
    publishes Actuator messages to four actuator_cmd topics.

    Exposes:
      - state_p1, state_p2, state_p3, state_e : numpy arrays with 13 elements
      - publish_rudder(pub, rudder_rad) to publish raw rudder in radians
    """
    def __init__(self):
        super().__init__('vessel_bridge')

        # Publishers for actuator commands
        self.pub_evader = self.create_publisher(Actuator, '/evader_03/actuator_cmd', 10)
        self.pub_p1 = self.create_publisher(Actuator, '/sookshma_00/actuator_cmd', 10)
        self.pub_p2 = self.create_publisher(Actuator, '/sookshma2_01/actuator_cmd', 10)
        self.pub_p3 = self.create_publisher(Actuator, '/sookshma3_02/actuator_cmd', 10)

        # Subscribers for vessel states (std_msgs/Float64MultiArray)
        self.sub_p1 = self.create_subscription(
            Odometry, '/sookshma_00/odometry_sim', self.p1_callback, 10)
        self.sub_p2 = self.create_subscription(
            Odometry, '/sookshma2_01/odometry_sim', self.p2_callback, 10)
        self.sub_p3 = self.create_subscription(
            Odometry, '/sookshma3_02/odometry_sim', self.p3_callback, 10)
        self.sub_e = self.create_subscription(
            Odometry, '/evader_03/odometry_sim', self.e_callback, 10)
        # self.odom_topic = f'{self.vessel.vessel_name}_
        # {self.vessel.vessel_id:02d}/odometry_sim'
        # latest states (None until message received)
        self.state_p1 = None
        self.state_p2 = None
        self.state_p3 = None
        self.state_e  = None

    # Callbacks convert Float64MultiArray.data -> numpy array (13 elements)
    def p1_callback(self, msg):
        # current_time = self.get_clock().now()
        # t = (current_time - self.start_time).nanoseconds / 1e9
        # Extract orientation quaternion
        self.get_logger().info("Received odometry for pursuer 1")
        quat = np.array([
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
        ])
        # Convert quaternion to Euler angles
        eul = quat_to_eul(quat, order='ZYX')
        self.state_p1=np.array([msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        eul[0],
        eul[1],
        eul[2],
        0.0 # rudder angle
        ])

    def p2_callback(self, msg):
        self.get_logger().info("Received odometry for pursuer 2")
        quat = np.array([
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
        ])
        # Convert quaternion to Euler angles
        eul = quat_to_eul(quat, order='ZYX')
        self.state_p2=np.array([msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        eul[0],
        eul[1],
        eul[2],
        0.0 # rudder angle
        ])

    def p3_callback(self, msg):
        self.get_logger().info("Received odometry for pursuer 3")
        quat = np.array([
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
        ])
        # Convert quaternion to Euler angles
        eul = quat_to_eul(quat, order='ZYX')
        self.state_p3=np.array([msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        eul[0],
        eul[1],
        eul[2],
        0.0 # rudder angle
        ])

    def e_callback(self, msg):
        self.get_logger().info("Received odometry for pursuer 3")
        quat = np.array([
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z
        ])
        # Convert quaternion to Euler angles
        eul = quat_to_eul(quat, order='ZYX')
        self.state_p3=np.array([msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z,
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        eul[0],
        eul[1],
        eul[2],
        0.0 # rudder angle
        ])
        
    # Publish raw rudder in radians (user said rudder is radian)
    def publish_rudder(self, pub, rudder_rad):
        msg = Actuator()
        # We send the raw radian value as requested
        msg.actuator_values = [float(rudder_rad)]
        msg.actuator_names = ['cs_1']
        msg.covariance = [0.0]
        pub.publish(msg)


# ===================== SIMULATION FUNCTION =====================
def run_simulation_with_live_vis(ros_bridge):

    NP = 10
    NC = 4
    Q = np.array([10, 10, 1])
    time_step = 0.1
    simulation_time = 100
    n_steps = int(simulation_time / time_step) + 1
    time = np.linspace(0, simulation_time, n_steps)
    # evader_x, evader_y = 50, 50

    # # --- Formation geometry ---
    # L = 15
    # angle_offset = np.deg2rad(120)
    # p1_x = evader_x + L * np.cos(angle_offset)
    # p1_y = evader_y + L * np.sin(angle_offset)
    # p2_x = evader_x + L * np.cos(angle_offset + 2 * np.pi / 3)
    # p2_y = evader_y + L * np.sin(angle_offset + 2 * np.pi / 3)
    # p3_x = evader_x + L * np.cos(angle_offset + 4 * np.pi / 3)
    # p3_y = evader_y + L * np.sin(angle_offset + 4 * np.pi / 3)

    # # --- Initial states (13-element) ---
    # X0_pursuer1 = np.array([0.4, 0.0, 0.0, 0.0, 0.01, 0.01, 10, 10, 0.0, 0.0, 0.0, 0.02, 0.01])
    # X0_pursuer2 = np.array([0.4, 0.01, 0.01, 0.01, 0.01, 0.01, 10, 20, 0.0, 0.0, 0.0, 0.01, 0.01])
    # X0_pursuer3 = np.array([0.4, 0.01, 0.01, 0.01, 0.01, 0.01, 7, 15, 0.0, 0.0, 0.0, 0, 0.01])
    # X0_evader   = np.array([0.5, 0.01, 0.01, 0.01, 0.01, 0.01, 10, 0, 0.0, 0.0, 0.0, 0.01, 0.01])

    obst_r = [0.1, 0.1]
    obs_pos = ([20, 20], [30, 40])
    goal_evader = np.array([10, 50])

    # # --- Buffers to store history ---
    # states_pursuer1 = [X0_pursuer1.copy()]
    # states_pursuer2 = [X0_pursuer2.copy()]
    # states_pursuer3 = [X0_pursuer3.copy()]
    # states_evader    = [X0_evader.copy()]  
    # Initialize state history from live ROS states
    print("Waiting for vessel state data...")
    while (ros_bridge.state_p1 is None or
           ros_bridge.state_p2 is None or
           ros_bridge.state_p3 is None or
           ros_bridge.state_e is None):
        rclpy.spin_once(ros_bridge, timeout_sec=0.2)
        print("Waiting for data from all vessels...")
    
    print("All vessel states received. Starting simulation...")
    states_pursuer1 = [ros_bridge.state_p1.copy()]
    states_pursuer2 = [ros_bridge.state_p2.copy()]
    states_pursuer3 = [ros_bridge.state_p3.copy()]
    states_evader = [ros_bridge.state_e.copy()]
    commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e = [], [], [], []
    commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e = [], [], [], []
    theta_G_history, theta_vals_history = [], []
    
    fig = plt.figure(figsize=(20, 10))
    evader_controller = controller(time_step, NP, NC, Q, obst_r)
    live_anim = PursuitAnimation(
        np.array(states_pursuer1),
        np.array(states_pursuer2),
        np.array(states_pursuer3),
        np.array(states_evader),
        time,
        obs_pos,
        obst_r,
        goal_evader,
        states_pursuer1[0],
        states_pursuer2[0],
        states_pursuer3[0],
        states_evader[0]

    )

    cooperative_controller = ApolloniusTradeoffController(desired_capture_distance=5.0)
    all_predicted_paths = []

    # Main simulation loop
    for t in tqdm(time[:-1], desc="Simulation Progress"):
        # Process incoming ROS messages (non-blocking)
        rclpy.spin_once(ros_bridge, timeout_sec=0.1)

        # Use the latest subscribed states when available, else use last-simulated state
        Xp1 = ros_bridge.state_p1.copy() if ros_bridge.state_p1 is not None else states_pursuer1[-1]
        Xp2 = ros_bridge.state_p2.copy() if ros_bridge.state_p2 is not None else states_pursuer2[-1]
        Xp3 = ros_bridge.state_p3.copy() if ros_bridge.state_p3 is not None else states_pursuer3[-1]
        Xe  = ros_bridge.state_e.copy()  if ros_bridge.state_e  is not None else states_evader[-1]

        control_e = np.zeros((NP, 2))
        pursuer_states = [Xp1, Xp2, Xp3]
        V_list = [.5, .5, .5]

        # --- Compute cooperative pursuit controls ---
        # Protect against zeros / invalid ratios by clipping
        def safe_theta(x_i, x_e):
            try:
                ratio = float(x_i[0]) / float(x_e[0])
                ratio = np.clip(ratio, -0.9999, 0.9999)
                return 2 * np.arcsin(ratio)
            except Exception:
                return 0.0

        theta_1 = safe_theta(Xp1, Xe)
        theta_2 = safe_theta(Xp2, Xe)
        theta_3 = safe_theta(Xp3, Xe)
        theta_vals = [theta_1, theta_2, theta_3]

        sorted_indices, sorted_polar = sort_pursuers_by_angle(pursuer_states, Xe)
        epsilons = compute_coverage_angles(sorted_polar, theta_vals)
        theta_G = compute_group_occupied_angle(theta_vals, epsilons)
        theta_G_history.append(theta_G)
        theta_vals_history.append(theta_vals.copy())

        d_c = 2.3
        lambda_min = 0.8
        R_p = 3.0
        R_o = 2.0
        R_b = 2.3

        cooperative_commands = cooperative_controller.compute_tradeoff_command(
            pursuer_states, Xe, V_list, theta_vals, d_c, lambda_min, R_p, R_o, R_b
        )

        control_p1 = np.array(velocity_to_controls(cooperative_commands[0], Xp1, Xe)).reshape(1, 2)
        control_p2 = np.array(velocity_to_controls(cooperative_commands[1], Xp2, Xe)).reshape(1, 2)
        control_p3 = np.array(velocity_to_controls(cooperative_commands[2], Xp3, Xe)).reshape(1, 2)

        nmpc_e, predicted_path = evader_controller.nlpsolve_with_cost(Xe, Xp1, Xp2, Xp3, goal_evader, time, obs_pos)
        all_predicted_paths.append(predicted_path)
        for i in range(NP):
            control_e[i] = [nmpc_e[i, 0], nmpc_e[i, 1]]

        # --- Clip controls to allowed ranges ---
        control_p1 = np.clip(control_p1, [0, -35], [2, 35])
        control_p2 = np.clip(control_p2, [0, -35], [2, 35])
        control_p3 = np.clip(control_p3, [0, -35], [2, 35])
        control_e  = np.clip(control_e,  [0, -35], [3, 35])

        # --- Save commanded control for analysis (rudder in radians, prop in RPM or whatever) ---
        commanded_rudders_p1.append(float(control_p1[0, 1]))
        commanded_rudders_p2.append(float(control_p2[0, 1]))
        commanded_rudders_p3.append(float(control_p3[0, 1]))
        commanded_rudders_e.append(float(control_e[0, 1]))

        commanded_props_p1.append(float(control_p1[0, 0]))
        commanded_props_p2.append(float(control_p2[0, 0]))
        commanded_props_p3.append(float(control_p3[0, 0]))
        commanded_props_e.append(float(control_e[0, 0]))

        # --- Publish rudder commands (raw radians) ---
        try:
            ros_bridge.publish_rudder(ros_bridge.pub_evader, control_e[0, 1])
            ros_bridge.publish_rudder(ros_bridge.pub_p1, control_p1[0, 1])
            ros_bridge.publish_rudder(ros_bridge.pub_p2, control_p2[0, 1])
            ros_bridge.publish_rudder(ros_bridge.pub_p3, control_p3[0, 1])
        except Exception as e:
            ros_bridge.get_logger().warning(f"publish_rudder failed: {e}")

        # --- Update vehicle states: prefer subscribed live state; if none, fallback to simulation step ---
        if ros_bridge.state_p1 is not None:
            new_state_p1 = ros_bridge.state_p1.copy()
            print("Using ROS-bridged pursuer ________________1 statefor pursuer 1")
        # else:
        #     new_state_p1 = simulation(Xp1, control_p1, time_step, flag=False)
        #     print("Using simulated pursuer ________________1 state")

        if ros_bridge.state_p2 is not None:
            new_state_p2 = ros_bridge.state_p2.copy()
            print("Using ROS-bridged pursuer 2 statefor pursuer 2")
        # else:
        #     new_state_p2 = simulation(Xp2, control_p2, time_step, flag=False)
        #     print("Using simulated pursuer 2 state")

        if ros_bridge.state_p3 is not None:
            new_state_p3 = ros_bridge.state_p3.copy()
            print("Using ROS-bridged pursuer 3 statefor pursuer 3")
        # else:
        #     new_state_p3 = simulation(Xp3, control_p3, time_step, flag=False)
        #     print("Using simulated pursuer __________3 state")

        if ros_bridge.state_e is not None:
            new_state_e = ros_bridge.state_e.copy()
            print("Using ROS-bridged evader statefor evader")
            Xe[0]=0.6
            new_state_e = simulation(Xe, control_e, time_step, flag=False)
            print("Using simulated evader state")

        # Append to history arrays
        states_pursuer1.append(new_state_p1)
        states_pursuer2.append(new_state_p2)
        states_pursuer3.append(new_state_p3)
        states_evader.append(new_state_e)

        # Update live visualization
        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t, predicted_path=predicted_path
        )

        # Small pause to allow plotting and CPU breathing
        plt.pause(0.01)

    # End loop
    plt.ioff()
    plt.close('all')

    return (
        np.array(states_pursuer1), np.array(states_pursuer2), np.array(states_pursuer3), np.array(states_evader),
        time, obs_pos, obst_r, goal_evader, states_pursuer1[0], states_pursuer2[0], states_pursuer3[0], states_evader[0],
        commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e,
        commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e, all_predicted_paths
    )


# ===================== MAIN =====================
if __name__ == "__main__":
    # Init ROS 2
    rclpy.init()
    ros_bridge = VesselROSBridge()

    plots_dir = "cooperative_21"
    os.makedirs(plots_dir, exist_ok=True)
    print(f"Created directory: {plots_dir}")
    print("Starting simulation...")

    simulation_results = run_simulation_with_live_vis(ros_bridge)

    # Shutdown ros node cleanly
    try:
        ros_bridge.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

    print("Simulation complete. Creating animation...")
    (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
     time, obs_pos, obst_r, goal_evader, X0_p1, X0_p2, X0_p3, X0_e,
     cmd_rud_p1, cmd_rud_p2, cmd_rud_p3, cmd_rud_e,
     cmd_prop_p1, cmd_prop_p2, cmd_prop_p3, cmd_prop_e,
     all_pred_path) = simulation_results

    animation_path = os.path.join(plots_dir, 'pursuit_animation_formation.gif')
    animator = PursuitAnimation(states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
                                time, obs_pos, obst_r, goal_evader, X0_p1, X0_p2, X0_p3, X0_e,
                                predicted_path=all_pred_path)
    animator.create_animation(animation_path)
    print("Animation saved as 'pursuit_animation_formation.gif'")

    print("Generating analysis plots...")
    generate_analysis_plots(states_pursuer1, states_pursuer2, states_pursuer3,
                            states_evader, cmd_rud_p1, cmd_rud_p2, cmd_rud_p3, cmd_rud_e,
                            cmd_prop_p1, cmd_prop_p2, cmd_prop_p3, cmd_prop_e,
                            plots_dir=plots_dir)
    print("Analysis plots generated and saved.")
    plt.close('all')
