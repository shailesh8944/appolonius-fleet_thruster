import os
import numpy as np
from tqdm import tqdm
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator

from module_kinematics import quat_to_eul
from control_utils import velocity_to_controls
from cooperative_pursuer import (
    ApolloniusTradeoffController,
    compute_group_occupied_angle,
    compute_coverage_angles,
    sort_pursuers_by_angle,
    check_capture,
)
from visulization import PursuitAnimation
from plot_analysis import (
    generate_analysis_plots,
    generate_evader_strategy_plots,
)
from Evader_escape_strategy import compute_evader_heading_ramana
from rudder_control import generate_evasion_rudder


class VesselROSBridge(Node):
    """
    Minimal ROS2 bridge: subscribes to odometry for all four vessels and
    publishes rudder commands.
    """

    def __init__(self):
        super().__init__("vessel_bridge_apf")

        self.pub_evader = self.create_publisher(Actuator, "/evader_03/actuator_cmd", 10)
        self.pub_p1 = self.create_publisher(Actuator, "/sookshma_00/actuator_cmd", 10)
        self.pub_p2 = self.create_publisher(Actuator, "/sookshma2_01/actuator_cmd", 10)
        self.pub_p3 = self.create_publisher(Actuator, "/sookshma3_02/actuator_cmd", 10)

        self.sub_p1 = self.create_subscription(
            Odometry, "/sookshma_00/odometry_sim", self.p1_callback, 10
        )
        self.sub_p2 = self.create_subscription(
            Odometry, "/sookshma2_01/odometry_sim", self.p2_callback, 10
        )
        self.sub_p3 = self.create_subscription(
            Odometry, "/sookshma3_02/odometry_sim", self.p3_callback, 10
        )
        self.sub_e = self.create_subscription(
            Odometry, "/evader_03/odometry_sim", self.e_callback, 10
        )

        self.state_p1 = None
        self.state_p2 = None
        self.state_p3 = None
        self.state_e = None

    def _odom_to_state(self, msg):
        quat = np.array(
            [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        eul = quat_to_eul(quat, order="ZYX")
        return np.array(
            [
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
                0.0,  # rudder angle placeholder
            ]
        )

    def p1_callback(self, msg):
        self.state_p1 = self._odom_to_state(msg)

    def p2_callback(self, msg):
        self.state_p2 = self._odom_to_state(msg)

    def p3_callback(self, msg):
        self.state_p3 = self._odom_to_state(msg)

    def e_callback(self, msg):
        self.state_e = self._odom_to_state(msg)

    def publish_rudder(self, pub, rudder_deg):
        msg = Actuator()
        msg.actuator_values = [float(rudder_deg)]
        msg.actuator_names = ["cs_1"]
        msg.covariance = [0.0]
        pub.publish(msg)


def run_simulation_with_live_vis(ros_bridge):
    time_step = 0.1
    simulation_time = 40
    n_steps = int(simulation_time / time_step) + 1
    time = np.linspace(0, simulation_time, n_steps)

    obst_r = [0.1, 0.1]
    obs_pos = ([20, 20], [30, 40])
    goal_evader = np.array([60, 25])

    print("Waiting for vessel state data...")
    while (
        ros_bridge.state_p1 is None
        or ros_bridge.state_p2 is None
        or ros_bridge.state_p3 is None
        or ros_bridge.state_e is None
    ):
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
    evader_diag_history = []

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
        states_evader[0],
    )

    cooperative_controller = ApolloniusTradeoffController(desired_capture_distance=5.0)
    all_predicted_paths = []

    for t in tqdm(time[:-1], desc="Simulation Progress"):
        rclpy.spin_once(ros_bridge, timeout_sec=0.1)

        Xp1 = ros_bridge.state_p1.copy() if ros_bridge.state_p1 is not None else states_pursuer1[-1]
        Xp2 = ros_bridge.state_p2.copy() if ros_bridge.state_p2 is not None else states_pursuer2[-1]
        Xp3 = ros_bridge.state_p3.copy() if ros_bridge.state_p3 is not None else states_pursuer3[-1]
        Xe = ros_bridge.state_e.copy() if ros_bridge.state_e is not None else states_evader[-1]

        pursuer_states = [Xp1, Xp2, Xp3]
        V_list = [abs(Xp1[0]), abs(Xp2[0]), abs(Xp3[0])]

        def safe_theta(x_i, x_e):
            try:
                ratio = float(x_i[0]) / float(x_e[0])
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

        R_o = 2.0
        R_b = 2.3
        cooperative_commands = cooperative_controller.compute_tradeoff_command(
            pursuer_states, Xe, V_list, theta_vals, R_o, R_b
        )

        control_p1 = np.array(velocity_to_controls(cooperative_commands[0], Xp1, Xe)).reshape(1, 2)
        control_p2 = np.array(velocity_to_controls(cooperative_commands[1], Xp2, Xe)).reshape(1, 2)
        control_p3 = np.array(velocity_to_controls(cooperative_commands[2], Xp3, Xe)).reshape(1, 2)

        apf_rudder_deg = float(generate_evasion_rudder(Xe, pursuer_states, goal_evader))
        control_p1 = np.clip(control_p1, [0, -35], [2, 35])
        control_p2 = np.clip(control_p2, [0, -35], [2, 35])
        control_p3 = np.clip(control_p3, [0, -35], [2, 35])
        apf_rudder_deg = float(np.clip(apf_rudder_deg, -35.0, 35.0))

        commanded_rudders_p1.append(float(control_p1[0, 1]))
        commanded_rudders_p2.append(float(control_p2[0, 1]))
        commanded_rudders_p3.append(float(control_p3[0, 1]))
        commanded_rudders_e.append(apf_rudder_deg)

        commanded_props_p1.append(float(control_p1[0, 0]))
        commanded_props_p2.append(float(control_p2[0, 0]))
        commanded_props_p3.append(float(control_p3[0, 0]))
        commanded_props_e.append(0.0)

        try:
            ros_bridge.publish_rudder(ros_bridge.pub_evader, apf_rudder_deg)
            ros_bridge.publish_rudder(ros_bridge.pub_p1, control_p1[0, 1])
            ros_bridge.publish_rudder(ros_bridge.pub_p2, control_p2[0, 1])
            ros_bridge.publish_rudder(ros_bridge.pub_p3, control_p3[0, 1])
        except Exception as e:
            ros_bridge.get_logger().warning(f"publish_rudder failed: {e}")

        new_state_p1 = Xp1 if ros_bridge.state_p1 is None else ros_bridge.state_p1.copy()
        new_state_p2 = Xp2 if ros_bridge.state_p2 is None else ros_bridge.state_p2.copy()
        new_state_p3 = Xp3 if ros_bridge.state_p3 is None else ros_bridge.state_p3.copy()
        new_state_e = Xe if ros_bridge.state_e is None else ros_bridge.state_e.copy()

        states_pursuer1.append(new_state_p1)
        states_pursuer2.append(new_state_p2)
        states_pursuer3.append(new_state_p3)
        states_evader.append(new_state_e)

        captured, i_cap, d_min = check_capture(pursuer_states, new_state_e, d_c=1)
        if captured:
            print(f"Captured by pursuer {i_cap}, distance = {d_min:.2f} m")

        psi_e, evader_debug = compute_evader_heading_ramana(
            pursuer_states, new_state_e, return_debug=True
        )
        evader_debug["time"] = float(t)
        evader_debug["apf_rudder_deg"] = apf_rudder_deg
        evader_debug["psi_apf"] = float(new_state_e[11])
        evader_diag_history.append(evader_debug)

        live_anim.live_update(
            np.array(states_pursuer1),
            np.array(states_pursuer2),
            np.array(states_pursuer3),
            np.array(states_evader),
            t,
        )
        plt.pause(0.01)

    plt.ioff()
    plt.close("all")

    return (
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
        states_evader[0],
        commanded_rudders_p1,
        commanded_rudders_p2,
        commanded_rudders_p3,
        commanded_rudders_e,
        commanded_props_p1,
        commanded_props_p2,
        commanded_props_p3,
        commanded_props_e,
        all_predicted_paths,
        evader_diag_history,
    )


if __name__ == "__main__":
    rclpy.init()
    ros_bridge = VesselROSBridge()

    plots_dir = "apf_evader_runs1"
    os.makedirs(plots_dir, exist_ok=True)
    print(f"Created directory: {plots_dir}")
    print("Starting APF-based evader simulation...")

    simulation_results = run_simulation_with_live_vis(ros_bridge)

    try:
        ros_bridge.destroy_node()
    except Exception:
        pass
    rclpy.shutdown()

    (
        states_pursuer1,
        states_pursuer2,
        states_pursuer3,
        states_evader,
        time,
        obs_pos,
        obst_r,
        goal_evader,
        X0_p1,
        X0_p2,
        X0_p3,
        X0_e,
        cmd_rud_p1,
        cmd_rud_p2,
        cmd_rud_p3,
        cmd_rud_e,
        cmd_prop_p1,
        cmd_prop_p2,
        cmd_prop_p3,
        cmd_prop_e,
        all_pred_path,
        evader_diag_history,
    ) = simulation_results

    animation_path = os.path.join(plots_dir, "pursuit_animation_apf.gif")
    animator = PursuitAnimation(
        states_pursuer1,
        states_pursuer2,
        states_pursuer3,
        states_evader,
        time,
        obs_pos,
        obst_r,
        goal_evader,
        X0_p1,
        X0_p2,
        X0_p3,
        X0_e,
        predicted_path=all_pred_path,
    )
    animator.create_animation(animation_path)
    print("Animation saved as 'pursuit_animation_apf.gif'")

    print("Generating analysis plots...")
    time_step = float(time[1] - time[0]) if len(time) > 1 else 0.1
    generate_analysis_plots(
        states_pursuer1,
        states_pursuer2,
        states_pursuer3,
        states_evader,
        cmd_rud_p1,
        cmd_rud_p2,
        cmd_rud_p3,
        cmd_rud_e,
        cmd_prop_p1,
        cmd_prop_p2,
        cmd_prop_p3,
        cmd_prop_e,
        plots_dir=plots_dir,
    )
    generate_evader_strategy_plots(
        evader_diag_history,
        plots_dir=plots_dir,
        time_step=time_step,
        evader_heading=states_evader[:, 11] if len(states_evader) else None,
    )
    print("Analysis plots generated and saved.")
    plt.close("all")
