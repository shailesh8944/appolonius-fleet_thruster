import numpy as np
import os
import time
import threading
from tqdm import tqdm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from interfaces.msg import Actuator

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from module_kinematics import quat_to_eul
from cooperative_pursuer import (
    ApolloniusTradeoffController,
    compute_group_occupied_angle,
    compute_coverage_angles,
    sort_pursuers_by_angle,
    check_capture
)
from mpc_evader import controller
from thrust_velocity import velocity_to_controls1
from control_utils import velocity_to_controls
from Evader_escape_strategy import compute_evader_heading_ramana
from visulization import PursuitAnimation
from plot_analysis import generate_analysis_plots, generate_evader_strategy_plots


# ===================== ROS 2 Bridge Node =====================
class VesselROSBridge(Node):
    def __init__(self):
        super().__init__('vessel_bridge')

        # -------- QoS: BEST_EFFORT + depth=1 to avoid blocking publishers --------
        # self.qos_odom = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1,
        #     durability=DurabilityPolicy.VOLATILE
        # )
        self.qos_odom = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,   # <-- IMPORTANT
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

        # Publishers
        self.pub_evader = self.create_publisher(Actuator, '/evader_03/actuator_cmd', 10)
        self.pub_p1     = self.create_publisher(Actuator, '/sookshma_00/actuator_cmd', 10)
        self.pub_p2     = self.create_publisher(Actuator, '/sookshma2_01/actuator_cmd', 10)
        self.pub_p3     = self.create_publisher(Actuator, '/sookshma3_02/actuator_cmd', 10)

        # Subscribers (Odometry)
        self.sub_p1 = self.create_subscription(Odometry, '/sookshma_00/odometry_sim', self.p1_callback, self.qos_odom)
        self.sub_p2 = self.create_subscription(Odometry, '/sookshma2_01/odometry_sim', self.p2_callback, self.qos_odom)
        self.sub_p3 = self.create_subscription(Odometry, '/sookshma3_02/odometry_sim', self.p3_callback, self.qos_odom)
        self.sub_e  = self.create_subscription(Odometry, '/evader_03/odometry_sim', self.e_callback,  self.qos_odom)

        # Latest states (shared with main thread) + lock
        self._lock = threading.Lock()
        self.state_p1 = None
        self.state_p2 = None
        self.state_p3 = None
        self.state_e  = None

        self.get_logger().info("VesselROSBridge started (BEST_EFFORT odom).")

    def _odom_to_state(self, msg: Odometry):
        # Your quat_to_eul expects [w, x, y, z]
        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ], dtype=float)

        eul = quat_to_eul(quat, order='ZYX')  # [roll, pitch, yaw] (your function)
        state = np.array([
            msg.twist.twist.linear.x,
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
            0.0  # rudder angle
        ], dtype=float)
        return state

    def p1_callback(self, msg: Odometry):
        st = self._odom_to_state(msg)
        st[0] = 0.5  # force P1 surge
        with self._lock:
            self.state_p1 = st

    def p2_callback(self, msg: Odometry):
        st = self._odom_to_state(msg)
        with self._lock:
            self.state_p2 = st

    def p3_callback(self, msg: Odometry):
        st = self._odom_to_state(msg)
        with self._lock:
            self.state_p3 = st

    def e_callback(self, msg: Odometry):
        st = self._odom_to_state(msg)
        with self._lock:
            self.state_e = st

    def get_latest_states(self):
        """Thread-safe fetch copies of latest states."""
        with self._lock:
            p1 = None if self.state_p1 is None else self.state_p1.copy()
            p2 = None if self.state_p2 is None else self.state_p2.copy()
            p3 = None if self.state_p3 is None else self.state_p3.copy()
            e  = None if self.state_e  is None else self.state_e.copy()
        return p1, p2, p3, e

    # ---- Your publishing helpers (unchanged logic) ----
    def publish_rudder(self, pub, rudder_deg):
        msg = Actuator()
        msg.actuator_values = [float(rudder_deg)]
        msg.actuator_names = ['cs_1']
        msg.covariance = [0.0]
        pub.publish(msg)

    @staticmethod
    def evader_rudder_to_thrusters_simple(rudder_deg):
        rudder_deg = float(np.clip(rudder_deg, -35.0, 35.0))
        turn = rudder_deg / 35.0
        turn_mag = float(np.clip(abs(turn), 0.0, 1.0))
        if turn >= 0.0:
            port = 1.0
            stbd = 1.0 - turn_mag
        else:
            stbd = 1.0
            port = 1.0 - turn_mag
        return float(port), float(stbd)

    def publish_thrust_evader(self, thrusts):
        actuator_msg = Actuator()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.header.frame_id = "apollonius_mpc"
        T_port, T_stbd = float(thrusts[0]), float(thrusts[1])
        actuator_msg.actuator_values = [T_stbd, T_port]
        actuator_msg.actuator_names = ["th_stbd", "th_port"]
        actuator_msg.covariance = [0.01, 0.01]
        self.pub_evader.publish(actuator_msg)

    def publish_thrust_p1(self, thrusts):
        actuator_msg = Actuator()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.header.frame_id = "apollonius_mpc"
        T_port, T_stbd = float(thrusts[0]), float(thrusts[1])
        actuator_msg.actuator_values = [T_stbd, T_port]
        actuator_msg.actuator_names = ["th_stbd", "th_port"]
        actuator_msg.covariance = [0.01, 0.01]
        self.pub_p1.publish(actuator_msg)

    def publish_thrust_p2(self, thrusts):
        actuator_msg = Actuator()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.header.frame_id = "apollonius_mpc"
        T_port, T_stbd = float(thrusts[0]), float(thrusts[1])
        actuator_msg.actuator_values = [T_stbd, T_port]
        actuator_msg.actuator_names = ["th_stbd", "th_port"]
        actuator_msg.covariance = [0.01, 0.01]
        self.pub_p2.publish(actuator_msg)

    def publish_thrust_p3(self, thrusts):
        actuator_msg = Actuator()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.header.frame_id = "apollonius_mpc"
        T_port, T_stbd = float(thrusts[0]), float(thrusts[1])
        actuator_msg.actuator_values = [T_stbd, T_port]
        actuator_msg.actuator_names = ["th_stbd", "th_port"]
        actuator_msg.covariance = [0.01, 0.01]
        self.pub_p3.publish(actuator_msg)


def start_executor_thread(node: Node, num_threads: int = 2):
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()
    return executor, th


# ===================== SIMULATION FUNCTION =====================
def run_simulation_with_live_vis(ros_bridge: VesselROSBridge):
    NP = 6
    NC = 2
    Q = np.array([100, 800, 400])
    time_step = 0.1
    simulation_time = 300
    n_steps = int(simulation_time / time_step) + 1
    time_vec = np.linspace(0, simulation_time, n_steps)

    obst_r = [0.1, 0.1]
    obs_pos = ([20, 20], [30, 40])
    goal_evader = np.array([4, 2])

    # ---- Wait for initial states WITHOUT spin_once (executor thread is already spinning) ----
    print("Waiting for vessel state data...")
    t0 = time.time()
    while rclpy.ok():
        p1, p2, p3, e = ros_bridge.get_latest_states()
        if (p1 is not None) and (p2 is not None) and (p3 is not None) and (e is not None):
            break
        if time.time() - t0 > 5.0:
            print("Still waiting... (check odom topics)")
            t0 = time.time()
        time.sleep(0.05)

    print("All vessel states received. Starting simulation...")

    states_pursuer1 = [p1.copy()]
    states_pursuer2 = [p2.copy()]
    states_pursuer3 = [p3.copy()]
    states_evader   = [e.copy()]

    commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e = [], [], [], []
    commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e = [], [], [], []
    commanded_port_act_p1, commanded_stbd_act_p1 = [], []
    evader_diag_history = []
    all_predicted_paths = []
    theta_G_history, theta_vals_history = [], []

    fig = plt.figure(figsize=(20, 10))

    evader_controller = controller(time_step, NP, NC, Q, obst_r)
    live_anim = PursuitAnimation(
        np.array(states_pursuer1),
        np.array(states_pursuer2),
        np.array(states_pursuer3),
        np.array(states_evader),
        time_vec,
        obs_pos,
        obst_r,
        goal_evader,
        states_pursuer1[0],
        states_pursuer2[0],
        states_pursuer3[0],
        states_evader[0]
    )

    cooperative_controller = ApolloniusTradeoffController(desired_capture_distance=5.0)

    for t in tqdm(time_vec[:-1], desc="Simulation Progress"):
        # pull latest states (executor thread keeps updating)
        p1, p2, p3, e = ros_bridge.get_latest_states()

        Xp1 = p1 if p1 is not None else states_pursuer1[-1]
        Xp2 = p2 if p2 is not None else states_pursuer2[-1]
        Xp3 = p3 if p3 is not None else states_pursuer3[-1]
        Xe  = e  if e  is not None else states_evader[-1]

        # enforce fixed speeds as you do
        Xp1[0] = 0.5
        Xp3[0] = 0.5
        Xe[0]  = 0.52

        pursuer_states = [Xp1, Xp2, Xp3]
        V_list = [abs(Xp1[0]), abs(Xp2[0]), abs(Xp3[0])]

        def safe_theta(x_i, x_e):
            try:
                ratio = float(x_i[0]) / max(1e-6, float(x_e[0]))
                ratio = np.clip(ratio, -0.9999, 0.9999)
                return 2 * np.arcsin(ratio)
            except Exception:
                return 0.0

        theta_vals = [safe_theta(Xp1, Xe), safe_theta(Xp2, Xe), safe_theta(Xp3, Xe)]

        _, sorted_polar = sort_pursuers_by_angle(pursuer_states, Xe)
        epsilons = compute_coverage_angles(sorted_polar, theta_vals)
        theta_G = compute_group_occupied_angle(theta_vals, epsilons)
        theta_G_history.append(theta_G)
        theta_vals_history.append(theta_vals.copy())

        R_o, R_b = 2.0, 2.3
        cooperative_commands = cooperative_controller.compute_tradeoff_command(
            pursuer_states, Xe, V_list, theta_vals, R_o, R_b
        )

        T_port_p1, T_stbd_p1 = velocity_to_controls1(cooperative_commands[0], Xp1, Xe)
        control_p1 = np.array([[T_port_p1, T_stbd_p1]], dtype=float)

        control_p2 = np.array(velocity_to_controls(cooperative_commands[1], Xp2, Xe)).reshape(1, 2)
        T_port_p2, T_stbd_p2 = velocity_to_controls1(cooperative_commands[1], Xp2, Xe)

        T_port_p3, T_stbd_p3 = velocity_to_controls1(cooperative_commands[2], Xp3, Xe)
        control_p3 = np.array([[T_port_p3, T_stbd_p3]], dtype=float)

        _, evader_debug = compute_evader_heading_ramana(pursuer_states, Xe, return_debug=True)
        evader_debug['time'] = float(t)
        evader_diag_history.append(evader_debug)

        nmpc_e, predicted_path = evader_controller.nlpsolve_with_cost(Xe, Xp1, Xp2, Xp3, goal_evader, time_vec, obs_pos)
        all_predicted_paths.append(predicted_path)

        control_e = np.clip(nmpc_e, -35, 35)

        # simple mapping for evader thrusters
        act_port_e, act_stbd_e = ros_bridge.evader_rudder_to_thrusters_simple(control_e[0])

        # store commands
        commanded_rudders_p1.append(float(control_p1[0, 1]) if control_p1.shape[1] > 1 else 0.0)
        commanded_rudders_p2.append(float(control_p2[0, 1]) if control_p2.shape[1] > 1 else 0.0)
        commanded_rudders_p3.append(float(control_p3[0, 1]) if control_p3.shape[1] > 1 else 0.0)
        commanded_rudders_e.append(float(control_e[0]))

        commanded_props_p1.append(float(control_p1[0, 0]))
        commanded_props_p2.append(float(control_p2[0, 0]))
        commanded_props_p3.append(float(control_p3[0, 0]))
        commanded_props_e.append(0.0)

        commanded_port_act_p1.append(float(T_port_p1))
        commanded_stbd_act_p1.append(float(T_stbd_p1))

        # publish commands (lightweight)
        try:
            ros_bridge.publish_rudder(ros_bridge.pub_evader, float(control_e[0]))
            ros_bridge.publish_thrust_evader([act_port_e, act_stbd_e])
            ros_bridge.publish_thrust_p1([T_port_p1, T_stbd_p1])
            ros_bridge.publish_rudder(ros_bridge.pub_p2, float(control_p2[0, 1]) if control_p2.shape[1] > 1 else 0.0)
            ros_bridge.publish_thrust_p2([T_port_p2, T_stbd_p2])
            ros_bridge.publish_thrust_p3([T_port_p3, T_stbd_p3])
        except Exception as ex:
            ros_bridge.get_logger().warn(f"Publishing actuator commands failed: {ex}")

        # append states
        states_pursuer1.append(Xp1.copy())
        states_pursuer2.append(Xp2.copy())
        states_pursuer3.append(Xp3.copy())
        states_evader.append(Xe.copy())

        captured, i_cap, d_min = check_capture(pursuer_states, Xe, d_c=1)
        if captured:
            print(f"Captured by pursuer {i_cap}, distance = {d_min:.2f} m")

        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t
        )

        plt.pause(0.001)

    plt.ioff()
    plt.close('all')

    return (
        np.array(states_pursuer1), np.array(states_pursuer2), np.array(states_pursuer3), np.array(states_evader),
        time_vec, obs_pos, obst_r, goal_evader, states_pursuer1[0], states_pursuer2[0], states_pursuer3[0], states_evader[0],
        commanded_rudders_p1, commanded_rudders_p2, commanded_rudders_p3, commanded_rudders_e,
        commanded_props_p1, commanded_props_p2, commanded_props_p3, commanded_props_e, all_predicted_paths,
        evader_diag_history, commanded_port_act_p1, commanded_stbd_act_p1
    )


# ===================== MAIN =====================
if __name__ == "__main__":
    rclpy.init()
    ros_bridge = VesselROSBridge()

    # IMPORTANT: spin in background so subscriptions don't choke publishers
    executor, spin_thread = start_executor_thread(ros_bridge, num_threads=2)

    plots_dir = "with_two_pursuer_experiment_01"
    os.makedirs(plots_dir, exist_ok=True)
    print(f"Created directory: {plots_dir}")
    print("Starting simulation...")

    simulation_results = run_simulation_with_live_vis(ros_bridge)

    # cleanup
    try:
        executor.shutdown()
    except Exception:
        pass
    try:
        ros_bridge.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

    print("Simulation complete. Creating animation...")

    (states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
     time_vec, obs_pos, obst_r, goal_evader, X0_p1, X0_p2, X0_p3, X0_e,
     cmd_rud_p1, cmd_rud_p2, cmd_rud_p3, cmd_rud_e,
     cmd_prop_p1, cmd_prop_p2, cmd_prop_p3, cmd_prop_e,
     all_pred_path, evader_diag_history, cmd_port_act_p1, cmd_stbd_act_p1) = simulation_results

    animation_path = os.path.join(plots_dir, 'pursuit_animation_formation.gif')
    animator = PursuitAnimation(states_pursuer1, states_pursuer2, states_pursuer3, states_evader,
                                time_vec, obs_pos, obst_r, goal_evader, X0_p1, X0_p2, X0_p3, X0_e,
                                predicted_path=all_pred_path)
    animator.create_animation(animation_path)
    print(f"Animation saved: {animation_path}")

    print("Generating analysis plots...")
    time_step = float(time_vec[1] - time_vec[0]) if len(time_vec) > 1 else 0.1
    generate_analysis_plots(states_pursuer1, states_pursuer2, states_pursuer3,
                            states_evader, cmd_rud_p1, cmd_rud_p2, cmd_rud_p3, cmd_rud_e,
                            cmd_prop_p1, cmd_prop_p2, cmd_prop_p3, cmd_prop_e,
                            plots_dir=plots_dir, port_act_p1=cmd_port_act_p1,
                            stbd_act_p1=cmd_stbd_act_p1, time_step=time_step)
    generate_evader_strategy_plots(evader_diag_history, plots_dir=plots_dir, time_step=time_step,
                                   evader_heading=states_evader[:, 11] if len(states_evader) else None)
    print("Analysis plots generated and saved.")
    plt.close('all')
