
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
import numpy as np
import math

class ONRTBoat(Node):
    def __init__(self):
        super().__init__('onrt_boat_controller')

        # Publishers
        self.actuator_publisher = self.create_publisher(Actuator, '/sookshma_00/actuator_cmd', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/er/odometry', self.pose_cb, 10)
        self.create_subscription(Pose2D, '/pursuer1/pose', self.pursuer1_pose_callback, 10)
        self.create_subscription(Pose2D, '/pursuer2/pose', self.pursuer2_pose_callback, 10)
        self.create_subscription(Pose2D, '/pursuer3/pose', self.pursuer3_pose_callback, 10)
        self.create_subscription(Float32, '/kf/yaw', self.kf_yaw_callback, 10)
        # Internal states
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.r = 0.0

        # Pursuer tracking
        self.pursuer1_pose = None
        self.pursuer2_pose = None
        self.pursuer3_pose = None

        # Goal position
        self.goal = np.array([10, 13])  # Fixed goal

        # Control parameters
        self.RPM = 450.0
        self.MAX_RUDDER_DEG = 35.0
        self.kp_angular = -1.5
        self.kd_angular = 0.5
        
        # Start control loop
        self.create_timer(0.1, self.control_loop)

    def pose_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.r = msg.twist.twist.angular.z

        # q = msg.pose.pose.orientation
        # self.yaw = self.quaternion_to_euler([q.x, q.y, q.z, q.w])[2]

    def quaternion_to_euler(self, quat):
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch = math.asin(t2) 

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
    def kf_yaw_callback(self, msg: Float32):
        self.yaw = msg.data
        self.get_logger().debug(f"Received EKF yaw: {math.degrees(self.yaw):.2f}°")
    def pursuer1_pose_callback(self, msg): self.pursuer1_pose = msg
    def pursuer2_pose_callback(self, msg): self.pursuer2_pose = msg
    def pursuer3_pose_callback(self, msg): self.pursuer3_pose = msg

    def control_loop(self):
        # Wait for valid pose
        if self.x == 0.0 and self.y == 0.0:
            self.get_logger().warn("Waiting for initial odometry...")
            return

        # Distance to goal
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        dist_to_goal = math.hypot(dx, dy)

        # Stop if close to goal
        if dist_to_goal < 2.0:
            self.stop()
            return

        rudder_deg = self.calculate_rudder()
        rudder_deg = self.clip(rudder_deg, self.MAX_RUDDER_DEG)
        self.publish_actuator_command(self.RPM, rudder_deg)

    def calculate_rudder(self):
        pos = np.array([self.x, self.y])
        goal = self.goal

        # APF parameters
        K_att = 10
        K_rep_base = 4000.0
        d0 = 800.0
        eps = 1e-6
        max_tc = 3.0

        # Pursuer repulsion
        pursuer_positions = self.get_pursuer_positions()
        closest_pursuer_dist = float('inf')
        if pursuer_positions:
            closest_pursuer_dist = min(np.linalg.norm(p - pos) for p in pursuer_positions)

        # Attractive force
        attraction_weight = np.clip(closest_pursuer_dist / (2 * d0), 0.1, 1.0)
        F_att = K_att * attraction_weight * (goal - pos)

        # Repulsive force
        F_rep = np.zeros(2)
        for p_pos in pursuer_positions:
            diff = pos - p_pos
            dist = np.linalg.norm(diff)
            if dist < eps:
                continue
            t_c = min(dist / 1.0, max_tc)
            K_rep = K_rep_base * np.exp(-dist / d0) / (1.0 + 0.5 * t_c)
            if dist < d0:
                rep_mag = K_rep * (1.0 / dist - 1.0 / d0) / (dist ** 1.5)
                F_rep += rep_mag * (diff / dist)

        # Combine forces
        F_total = F_att + F_rep
        if np.linalg.norm(F_total) < eps:
            psi_des = self.yaw
        else:
            psi_des = np.arctan2(F_total[1], F_total[0])

        err = self.ssa(psi_des - self.yaw)

        emergency_turn = 0.0
        if closest_pursuer_dist < d0 / 4:
            emergency_turn = np.sign(err) * 0.5 * math.radians(self.MAX_RUDDER_DEG)

        # ...existing code...
        rudder = self.kp_angular * err - self.kd_angular * self.r + emergency_turn
        rudder_deg = math.degrees(rudder)
        rudder_deg = self.clip(rudder_deg, self.MAX_RUDDER_DEG)
        self.get_logger().info(
            f"Pos: ({self.x:.1f}, {self.y:.1f}) | Goal: ({goal[0]}, {goal[1]}) | "
            f"Heading: {math.degrees(self.yaw):.1f}°, Desired: {math.degrees(psi_des):.1f}°, "
            f"Rudder: {rudder_deg:.1f}°, Closest pursuer: {closest_pursuer_dist:.2f}"
        )

        return rudder_deg

    def get_pursuer_positions(self):
        positions = []
        for pursuer in [self.pursuer1_pose, self.pursuer2_pose, self.pursuer3_pose]:
            if pursuer is not None:
                positions.append(np.array([pursuer.x, pursuer.y]))
        return positions

    def ssa(self, ang):
        return (ang + np.pi) % (2 * np.pi) - np.pi

    def clip(self, value, threshold):
        return max(min(value, threshold), -threshold)

    def publish_actuator_command(self, rpm, rudder_deg):
        m = Actuator()
        m.header.stamp = self.get_clock().now().to_msg()
        m.actuator_names = ['rudder', 'propeller']
        m.actuator_values = [float(rudder_deg), float(rpm)]
        self.actuator_publisher.publish(m)

    def stop(self):
        self.publish_actuator_command(0.0, 0.0)
        self.get_logger().info("Goal reached. Vessel stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = ONRTBoat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
