#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from math import sin, cos
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Your custom actuator msg
from interfaces.msg import Actuator

# SBG custom IMU message
from sbg_driver.msg import SbgImuData  # NOTE: This matches your topic type

# ----------------------------
# Utility functions
# ----------------------------
def quaternion_to_euler(quat_xyzw):
    """
    Convert quaternion (x, y, z, w) -> (roll, pitch, yaw) in radians.
    """
    x, y, z, w = quat_xyzw
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw


def extract_sbg_quaternion_xyzw(msg: SbgImuData):
    """
    Try to extract quaternion (x,y,z,w) from SbgImuData across common sbg_driver versions.
    If your message layout differs, this function will raise AttributeError with a helpful hint.
    """
    # Common patterns seen in sbg_driver:
    # 1) msg.quat.x/y/z/w
    if hasattr(msg, "quat"):
        q = msg.quat
        return [float(q.x), float(q.y), float(q.z), float(q.w)]

    # 2) msg.orientation.x/y/z/w
    if hasattr(msg, "orientation"):
        q = msg.orientation
        return [float(q.x), float(q.y), float(q.z), float(q.w)]

    # 3) msg.ekf_quat or msg.ekfQuat (less common in IMU data msg)
    if hasattr(msg, "ekf_quat"):
        q = msg.ekf_quat
        return [float(q.x), float(q.y), float(q.z), float(q.w)]

    raise AttributeError(
        "Could not find quaternion in SbgImuData. "
        "Tried: msg.quat, msg.orientation, msg.ekf_quat. "
        "Run: `ros2 interface show sbg_driver/msg/SbgImuData` to see fields."
    )


def extract_sbg_gyro_z(msg: SbgImuData):
    """
    Extract yaw rate (r) around Z axis from SbgImuData across common versions.
    """
    # 1) msg.gyro.z
    if hasattr(msg, "gyro"):
        g = msg.gyro
        if hasattr(g, "z"):
            return float(g.z)

    # 2) msg.angular_velocity.z
    if hasattr(msg, "angular_velocity"):
        av = msg.angular_velocity
        if hasattr(av, "z"):
            return float(av.z)

    # 3) msg.rate or msg.gyro_z
    if hasattr(msg, "gyro_z"):
        return float(msg.gyro_z)

    raise AttributeError(
        "Could not find gyro z in SbgImuData. "
        "Tried: msg.gyro.z, msg.angular_velocity.z, msg.gyro_z. "
        "Run: `ros2 interface show sbg_driver/msg/SbgImuData` to see fields."
    )


# ----------------------------
# EKF Node
# ----------------------------
class SimpleEKFNode(Node):
    """
    State: x = [u, v, r, X, Y, psi]^T
      u,v,r : body velocities
      X,Y   : position in world
      psi   : heading (yaw)
    """
    def __init__(self):
        super().__init__('simple_ekf_node')

        # ----------------------------
        # EKF variables
        # ----------------------------
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 1000.0

        self.Q = np.diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])  # process noise
        self.R_uwb = np.diag([1.0, 1.0])                      # measurement noise UWB
        self.R_imu = np.diag([0.05, 0.05])                    # measurement noise IMU (r, psi)

        self.last_time = None
        self.last_imu_time = None
        self.last_uwb_time = None

        self.th_stbd = 0.0
        self.th_port = 0.0

        # Latest quaternion from SBG (xyzw)
        self.quat_xyzw = None

        # If we updated state recently
        self.state_updated = False

        # ----------------------------
        # QoS (sensor-friendly)
        # ----------------------------
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ----------------------------
        # Subscriptions
        # ----------------------------
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sookshma_04/uwb',
            self.uwb_callback,
            qos_sensor
        )

        self.sbg_imu_sub = self.create_subscription(
            SbgImuData,
            '/sookshma_04/sbg/imu/data',
            self.sbg_imu_callback,
            qos_sensor
        )

        self.actuator_sub = self.create_subscription(
            Actuator,
            '/sookshma_04/actuator_feedback',
            self.actuator_callback,
            10
        )

        # ----------------------------
        # Publishers
        # ----------------------------
        self.odom_pub = self.create_publisher(Odometry, '/er/odometry', 10)
        self.yaw_pub = self.create_publisher(Float64, '/er/yaw', 10)

        # Publish at fixed rate even if callbacks are slow
        self.publish_rate = 10.0  # Hz
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Debug timer so it never "looks stuck"
        self.debug_timer = self.create_timer(2.0, self.debug_status)

        self.get_logger().info("EKF node started. Waiting for /sookshma_04/uwb and /sookshma_04/sbg/imu/data ...")

    # ----------------------------
    # EKF steps
    # ----------------------------
    def predict(self, dt: float):
        # Simple kinematic propagation
        F = np.eye(6)

        psi = float(self.x[5, 0])
        F[3, 0] = dt * cos(psi)    # u contributes to X
        F[3, 1] = -dt * sin(psi)   # v contributes to X
        F[4, 0] = dt * sin(psi)    # u contributes to Y
        F[4, 1] = dt * cos(psi)    # v contributes to Y

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def update_uwb(self, X_meas: float, Y_meas: float):
        z = np.array([[X_meas], [Y_meas]])
        H = np.zeros((2, 6))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        self.update(z, H, self.R_uwb)

    def update_imu(self, r_meas: float, psi_meas: float):
        z = np.array([[r_meas], [psi_meas]])
        H = np.zeros((2, 6))
        H[0, 2] = 1.0
        H[1, 5] = 1.0
        self.update(z, H, self.R_imu)

    # ----------------------------
    # Callbacks
    # ----------------------------
    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_uwb_time = now

        X_meas = float(msg.pose.pose.position.x)
        Y_meas = float(msg.pose.pose.position.y)

        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0.0:
                self.predict(dt)

        self.update_uwb(X_meas, Y_meas)

        self.last_time = now
        self.state_updated = True

    def sbg_imu_callback(self, msg: SbgImuData):
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_imu_time = now

        # predict using dt
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0.0:
                self.predict(dt)

        # Extract quaternion & yaw
        try:
            q_xyzw = extract_sbg_quaternion_xyzw(msg)
            self.quat_xyzw = q_xyzw
            _, _, psi = quaternion_to_euler(q_xyzw)
        except Exception as e:
            self.get_logger().error(f"SBG quaternion extraction failed: {e}")
            return

        # Extract yaw rate r (gyro z)
        try:
            r_meas = extract_sbg_gyro_z(msg)
        except Exception as e:
            self.get_logger().error(f"SBG gyro extraction failed: {e}")
            return

        # Update EKF
        self.update_imu(r_meas, psi)

        # Log
        self.get_logger().info(
            f"KF yaw={psi*180/np.pi:.2f} deg, r={r_meas:.3f} rad/s, "
            f"u={self.x[0,0]:.3f}, v={self.x[1,0]:.3f}, X={self.x[3,0]:.2f}, Y={self.x[4,0]:.2f}"
        )

        self.last_time = now
        self.state_updated = True

    def actuator_callback(self, msg: Actuator):
        if len(msg.actuator_values) >= 2:
            self.th_stbd = float(msg.actuator_values[0])
            self.th_port = float(msg.actuator_values[1])

    # ----------------------------
    # Timers / publishing
    # ----------------------------
    def timer_callback(self):
        if self.state_updated:
            self.publish_odom()
            self.publish_yaw()
            self.state_updated = False

    def debug_status(self):
        now = self.get_clock().now().nanoseconds / 1e9
        imu_age = (now - self.last_imu_time) if self.last_imu_time is not None else None
        uwb_age = (now - self.last_uwb_time) if self.last_uwb_time is not None else None

        if imu_age is None or imu_age > 2.0:
            self.get_logger().warn(f"No recent IMU data on /sookshma_04/sbg/imu/data (age={imu_age}).")
        if uwb_age is None or uwb_age > 2.0:
            self.get_logger().warn(f"No recent UWB data on /sookshma_04/uwb (age={uwb_age}).")

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'NED'  # keep your convention

        odom.pose.pose.position = Point(
            x=float(self.x[3, 0]),
            y=float(self.x[4, 0]),
            z=0.0
        )

        # Publish quaternion correctly as x,y,z,w
        if self.quat_xyzw is not None:
            qx, qy, qz, qw = self.quat_xyzw
            odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = float(self.x[0, 0])   # u
        odom.twist.twist.linear.y = float(self.x[1, 0])   # v
        odom.twist.twist.angular.z = float(self.x[2, 0])  # r

        self.odom_pub.publish(odom)

    def publish_yaw(self):
        if self.quat_xyzw is None:
            return
        _, _, yaw = quaternion_to_euler(self.quat_xyzw)

        yaw_msg = Float64()
        yaw_msg.data = float(yaw)  # radians (correct)
        self.yaw_pub.publish(yaw_msg)


def main():
    rclpy.init()
    node = SimpleEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
