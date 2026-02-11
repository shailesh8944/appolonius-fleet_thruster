import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from interfaces.msg import Actuator

from module_kinematics import quat_to_eul, ssa
from thrust_velocity import velocity_to_controls1


WAYPOINTS_FILE = os.environ.get(
    "APF_WAYPOINTS_FILE",
    os.path.join(os.path.dirname(__file__), "waypoints.yml"),
)


def load_waypoints(path):
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    waypoints = data.get("waypoints", {})
    params = data.get("params", {})
    return waypoints, params


class ILOSWaypointFollower:
    def __init__(
        self,
        waypoints,
        lookahead=5.0,
        acceptance_radius=1.0,
        loop=True,
        on_reach=None,
        ilos_gain=0.5,
        integral_limit=20.0,
    ):
        self.waypoints = np.asarray(waypoints, dtype=float)
        if self.waypoints.ndim != 2 or self.waypoints.shape[0] < 2:
            raise ValueError("Waypoints must be an (N,2) array with N >= 2.")

        self.lookahead = float(lookahead)
        self.acceptance_radius = float(acceptance_radius)
        self.loop = bool(loop)
        self.idx = 1
        self.on_reach = on_reach
        self.ilos_gain = float(ilos_gain)
        self.integral_limit = float(integral_limit)
        self._cte_int = 0.0

    def _advance(self):
        if self.on_reach is not None:
            self.on_reach(self.idx, self.waypoints[self.idx])
        if self.idx < len(self.waypoints) - 1:
            self.idx += 1
        elif self.loop:
            self.idx = 1
        self._cte_int = 0.0

    def guidance(self, position_xy, dt):
        pos = np.asarray(position_xy, dtype=float)
        target = self.waypoints[self.idx]
        if np.linalg.norm(pos - target) <= self.acceptance_radius:
            self._advance()
            target = self.waypoints[self.idx]

        prev = self.waypoints[self.idx - 1]
        path = target - prev
        path_len = np.linalg.norm(path)
        if path_len < 1e-6:
            heading = float(np.arctan2(target[1] - pos[1], target[0] - pos[0]))
            return ssa(heading), 0.0, self.idx, target

        t_hat = path / path_len
        rel = pos - prev
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        y_e = float(np.dot(rel, n_hat))
        if dt is not None and dt > 0.0:
            self._cte_int = float(
                np.clip(self._cte_int + y_e * dt, -self.integral_limit, self.integral_limit)
            )

        alpha = float(np.arctan2(t_hat[1], t_hat[0]))
        correction = float(np.arctan2(y_e + self.ilos_gain * self._cte_int, self.lookahead))
        heading = ssa(alpha - correction)
        return heading, y_e, self.idx, target

    def current_target(self):
        return self.idx, self.waypoints[self.idx]


class WaypointTrackingAll(Node):
    def __init__(self):
        super().__init__("waypoint_tracking_all")
        waypoints, params = load_waypoints(WAYPOINTS_FILE)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_p1 = self.create_publisher(Actuator, "/sookshma_00/actuator_cmd", 10)
        self.pub_p2 = self.create_publisher(Actuator, "/sookshma2_01/actuator_cmd", 10)
        self.pub_p3 = self.create_publisher(Actuator, "/sookshma3_02/actuator_cmd", 10)
        self.pub_e = self.create_publisher(Actuator, "/evader_03/actuator_cmd", 10)

        self.sub_p1 = self.create_subscription(Odometry, "/sookshma_00/odometry_sim", self.p1_callback, qos)
        self.sub_p2 = self.create_subscription(Odometry, "/sookshma2_01/odometry_sim", self.p2_callback, qos)
        self.sub_p3 = self.create_subscription(Odometry, "/sookshma3_02/odometry_sim", self.p3_callback, qos)
        self.sub_e = self.create_subscription(Odometry, "/evader_03/odometry_sim", self.e_callback, qos)

        self.state_p1 = None
        self.state_p2 = None
        self.state_p3 = None
        self.state_e = None

        lookahead = params.get("lookahead", 5.0)
        acceptance_radius = float(params.get("acceptance_radius", 1.0))
        loop = bool(params.get("loop", True))
        ilos_gain = float(params.get("ilos_gain", 0.5))
        integral_limit = float(params.get("ilos_integral_limit", 20.0))
        self.relative_waypoints = bool(params.get("relative_waypoints", True))

        self._total_waypoints = {
            "p1": len(waypoints.get("p1", [])),
            "p2": len(waypoints.get("p2", [])),
            "p3": len(waypoints.get("p3", [])),
            "evader": len(waypoints.get("evader", [])),
        }
        self._completed = {"p1": False, "p2": False, "p3": False, "evader": False}
        self._reached = {"p1": 0, "p2": 0, "p3": 0, "evader": 0}
        self._origin = {"p1": None, "p2": None, "p3": None, "evader": None}

        def make_on_reach(name):
            def _on_reach(idx, point):
                self._reached[name] = max(self._reached[name], idx)
                self.get_logger().info(
                    f"{name} reached waypoint {idx}/{self._total_waypoints[name]-1}: "
                    f"({point[0]:.2f}, {point[1]:.2f}) | "
                    f"tracked={self._reached[name]}/{self._total_waypoints[name]-1}"
                )
                if (not self._completed[name]) and (idx >= self._total_waypoints[name] - 1) and (not loop):
                    self._completed[name] = True
                    self.get_logger().info(f"{name} completed all waypoints.")
            return _on_reach

        def resolve_lookahead(name):
            if isinstance(lookahead, dict):
                if name in lookahead:
                    return float(lookahead[name])
                if "default" in lookahead:
                    return float(lookahead["default"])
                return 5.0
            return float(lookahead)

        self.follower_p1 = ILOSWaypointFollower(
            waypoints.get("p1"),
            resolve_lookahead("p1"),
            acceptance_radius,
            loop,
            on_reach=make_on_reach("p1"),
            ilos_gain=ilos_gain,
            integral_limit=integral_limit,
        )
        self.follower_p2 = ILOSWaypointFollower(
            waypoints.get("p2"),
            resolve_lookahead("p2"),
            acceptance_radius,
            loop,
            on_reach=make_on_reach("p2"),
            ilos_gain=ilos_gain,
            integral_limit=integral_limit,
        )
        self.follower_p3 = ILOSWaypointFollower(
            waypoints.get("p3"),
            resolve_lookahead("p3"),
            acceptance_radius,
            loop,
            on_reach=make_on_reach("p3"),
            ilos_gain=ilos_gain,
            integral_limit=integral_limit,
        )
        self.follower_e = ILOSWaypointFollower(
            waypoints.get("evader"),
            resolve_lookahead("evader"),
            acceptance_radius,
            loop,
            on_reach=make_on_reach("evader"),
            ilos_gain=ilos_gain,
            integral_limit=integral_limit,
        )

        speeds = params.get("speeds", {})
        self.speed_p1 = float(speeds.get("p1", 0.5))
        self.speed_p2 = float(speeds.get("p2", 0.5))
        self.speed_p3 = float(speeds.get("p3", 0.5))
        self.speed_e = float(speeds.get("evader", 0.5))

        self.dt = float(params.get("dt", 0.1))
        self._last_dist_log = {"p1": 0.0, "p2": 0.0, "p3": 0.0, "evader": 0.0}
        self._dist_log_period = float(params.get("dist_log_period", 1.0))
        self.timer = self.create_timer(self.dt, self.update_controls)

    @staticmethod
    def _odom_to_state(msg: Odometry):
        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ], dtype=float)
        eul = quat_to_eul(quat, order="ZYX")
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
            0.0,
        ], dtype=float)
        return state

    def p1_callback(self, msg: Odometry):
        self.state_p1 = self._odom_to_state(msg)

    def p2_callback(self, msg: Odometry):
        self.state_p2 = self._odom_to_state(msg)

    def p3_callback(self, msg: Odometry):
        self.state_p3 = self._odom_to_state(msg)

    def e_callback(self, msg: Odometry):
        self.state_e = self._odom_to_state(msg)

    def publish_thrust(self, pub, port, stbd):
        actuator_msg = Actuator()
        actuator_msg.header.stamp = self.get_clock().now().to_msg()
        actuator_msg.header.frame_id = "waypoint_tracking"
        actuator_msg.actuator_values = [float(stbd), float(port)]
        actuator_msg.actuator_names = ["th_stbd", "th_port"]
        actuator_msg.covariance = [0.01, 0.01]
        pub.publish(actuator_msg)

    def update_controls(self):
        if all(self._completed.values()) and self.timer is not None:
            self.get_logger().info("All vessels completed waypoints. Stopping controller.")
            self.timer.cancel()
            return
        if self.state_p1 is not None:
            pos = self.state_p1[6:8].copy()
            state = self.state_p1.copy()
            if self.relative_waypoints and self._origin["p1"] is None:
                self._origin["p1"] = pos.copy()
            if self._origin["p1"] is not None:
                pos_rel = pos - self._origin["p1"]
            else:
                pos_rel = pos
            heading, cte, idx, target = self.follower_p1.guidance(pos_rel, self.dt)
            dist = float(np.linalg.norm(pos_rel - target))
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_dist_log["p1"] >= self._dist_log_period:
                actual_heading = ssa(float(state[11]))
                self.get_logger().info(
                    f"p1 distance to wp {idx}: {dist:.2f} m | "
                    f"desired_heading={np.degrees(heading):.1f}deg "
                    f"actual_heading={np.degrees(actual_heading):.1f}deg "
                    f"cte={cte:.2f} m | "
                    f"tracked={self._reached['p1']}/{self._total_waypoints['p1']-1}"
                )
                self._last_dist_log["p1"] = now
            velocity = np.array([self.speed_p1 * np.cos(heading), self.speed_p1 * np.sin(heading)], dtype=float)
            port, stbd = velocity_to_controls1(velocity, state, state, dt=self.dt)
            self.publish_thrust(self.pub_p1, port, stbd)

        if self.state_p2 is not None:
            pos = self.state_p2[6:8].copy()
            state = self.state_p2.copy()
            if self.relative_waypoints and self._origin["p2"] is None:
                self._origin["p2"] = pos.copy()
            if self._origin["p2"] is not None:
                pos_rel = pos - self._origin["p2"]
            else:
                pos_rel = pos
            heading, cte, idx, target = self.follower_p2.guidance(pos_rel, self.dt)
            dist = float(np.linalg.norm(pos_rel - target))
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_dist_log["p2"] >= self._dist_log_period:
                actual_heading = ssa(float(state[11]))
                self.get_logger().info(
                    f"p2 distance to wp {idx}: {dist:.2f} m | "
                    f"desired_heading={np.degrees(heading):.1f}deg "
                    f"actual_heading={np.degrees(actual_heading):.1f}deg "
                    f"cte={cte:.2f} m | "
                    f"tracked={self._reached['p2']}/{self._total_waypoints['p2']-1}"
                )
                self._last_dist_log["p2"] = now
            velocity = np.array([self.speed_p2 * np.cos(heading), self.speed_p2 * np.sin(heading)], dtype=float)
            port, stbd = velocity_to_controls1(velocity, state, state, dt=self.dt)
            self.publish_thrust(self.pub_p2, port, stbd)

        if self.state_p3 is not None:
            pos = self.state_p3[6:8].copy()
            state = self.state_p3.copy()
            if self.relative_waypoints and self._origin["p3"] is None:
                self._origin["p3"] = pos.copy()
            if self._origin["p3"] is not None:
                pos_rel = pos - self._origin["p3"]
            else:
                pos_rel = pos
            heading, cte, idx, target = self.follower_p3.guidance(pos_rel, self.dt)
            dist = float(np.linalg.norm(pos_rel - target))
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_dist_log["p3"] >= self._dist_log_period:
                actual_heading = ssa(float(state[11]))
                self.get_logger().info(
                    f"p3 distance to wp {idx}: {dist:.2f} m | "
                    f"desired_heading={np.degrees(heading):.1f}deg "
                    f"actual_heading={np.degrees(actual_heading):.1f}deg "
                    f"cte={cte:.2f} m | "
                    f"tracked={self._reached['p3']}/{self._total_waypoints['p3']-1}"
                )
                self._last_dist_log["p3"] = now
            velocity = np.array([self.speed_p3 * np.cos(heading), self.speed_p3 * np.sin(heading)], dtype=float)
            port, stbd = velocity_to_controls1(velocity, state, state, dt=self.dt)
            self.publish_thrust(self.pub_p3, port, stbd)

        if self.state_e is not None:
            pos = self.state_e[6:8].copy()
            state = self.state_e.copy()
            if self.relative_waypoints and self._origin["evader"] is None:
                self._origin["evader"] = pos.copy()
            if self._origin["evader"] is not None:
                pos_rel = pos - self._origin["evader"]
            else:
                pos_rel = pos
            heading, cte, idx, target = self.follower_e.guidance(pos_rel, self.dt)
            dist = float(np.linalg.norm(pos_rel - target))
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_dist_log["evader"] >= self._dist_log_period:
                actual_heading = ssa(float(state[11]))
                self.get_logger().info(
                    f"evader distance to wp {idx}: {dist:.2f} m | "
                    f"desired_heading={np.degrees(heading):.1f}deg "
                    f"actual_heading={np.degrees(actual_heading):.1f}deg "
                    f"cte={cte:.2f} m | "
                    f"tracked={self._reached['evader']}/{self._total_waypoints['evader']-1}"
                )
                self._last_dist_log["evader"] = now
            velocity = np.array([self.speed_e * np.cos(heading), self.speed_e * np.sin(heading)], dtype=float)
            port, stbd = velocity_to_controls1(velocity, state, state, dt=self.dt)
            self.publish_thrust(self.pub_e, port, stbd)


def main():
    rclpy.init()
    node = WaypointTrackingAll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
