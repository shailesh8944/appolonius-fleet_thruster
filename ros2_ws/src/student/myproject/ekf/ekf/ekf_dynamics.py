# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped,Quaternion
# from std_msgs.msg import Float64
# from interfaces.msg import Actuator
# import numpy as np
# import sys

# sys.path.append('/workspaces/mavlab/')

# from ros2_ws.src.student.tut_03.read_input import read_input
# from ros2_ws.src.student.tut_03.class_vessel import Vessel
# from ros2_ws.src.student.tut_03.simulate import simulate
# from ros2_ws.src.student.tut_03.module_kinematics import eul_to_quat, eul_to_rotm, quat_to_eul, rotm_to_eul, ssa, clip
# from nav_msgs.msg import Odometry

# class EKF:
#     def __init__(self, sampling_rate, n_states=13, n_inp=1, logger=None):
#         self.dt = 1.0 / sampling_rate
#         self.n_states = n_states
#         self.n_inp = n_inp
#         self.x = np.zeros((n_states, 1))
#         self.P = np.eye(n_states) * 0.1
#         self.E = np.eye(n_states)
        
#         #self.Q = eps * np.eye(n_states)
#         self.Q = np.diag([5, 5, 5] + [5] * (n_states-3))
#         self.logger = logger
        

#     def jacobian(self, fun, x0):
#         x0 = x0.flatten()
#         f0 = fun(x0)
#         m, n = f0.size, x0.size
#         J = np.zeros((m, n))
#         eps = 1e-5
#         for i in range(n):
#             dx = np.zeros(n)
#             dx[i] = eps
#             f1 = fun(x0 + dx)
#             f2 = fun(x0 - dx)
#             J[:, i] = (f1 - f2) / (2 * eps)
#         return J

#     def predict(self, u, plant_model):
#         x0 = self.x.flatten()
#         dt = self.dt
#         f = lambda t, x: plant_model(t, x, u)
#         k1 = plant_model(0, x0)
#         k2 = plant_model(dt/2, x0 + dt/2 * k1)
#         k3 = plant_model(dt/2, x0 + dt/2 * k2)
#         k4 = plant_model(dt, x0 + dt * k3)
#         x_pred = x0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
#         F = self.jacobian(lambda x: f(0, x), x_pred)

#         try:
#             P_pred = F @ self.P @ F.T + self.E @ self.Q @ self.E.T
#         except np.linalg.LinAlgError as e:
#             if self.logger:
#                 self.logger.error(f"Prediction failed due to LinAlgError: {e}")
#             return

#         self.x = x_pred.reshape(-1, 1)
#         self.P = P_pred

#     def correct(self, y, C, R, meas_model):
#         I = np.eye(self.n_states)
#         eps = 1
#         R = R + eps * np.eye(R.shape[0])
#         S = C @ self.P @ C.T + R
#         try:
#             U, s, Vh = np.linalg.svd(S)
#             S_inv = Vh.T @ np.diag(1.0 / s) @ U.T
#         except np.linalg.LinAlgError as e:
#             if self.logger:
#                 self.logger.error(f"Correction failed due to LinAlgError: {e}")
#             return

#         K = self.P @ C.T @ S_inv
#         y = y.reshape(-1,1)
#         h = meas_model(self.x.flatten()).reshape(-1,1)
#         innov = y - h
#         dx = K @ innov
#         for i in range(3):
#             dx[i,0] = clip(dx[i,0], 1.0)
#         self.x += dx
#         self.P = (I - K @ C) @ self.P @ (I - K @ C).T + K @ R @ K.T
#         self.P = (self.P + self.P.T)/2
#         max_vel=0.5
#         max_rate=.1
#         self.x[0:3] = np.clip(self.x[0:3], -max_vel, max_vel)
#         self.x[3:6] = np.clip(self.x[3:6], -max_rate, max_rate)
#         for i in [9,10,11]:
#             self.x[i,0] = ssa(self.x[i,0])

# class SimpleEKFNode(Node):
#     def __init__(self):
#         super().__init__('simple_ekf_node')
#         self.create_subscription(Imu, '/sookshma_00/imu/data', self.imu_cb, 10)
#         self.create_subscription(PoseWithCovarianceStamped, '/sookshma_00/uwb', self.uwb_cb, 10)
#         self.create_subscription(Actuator, '/sookshma_00/actuator_cmd', self.actuator_cb, 10)
#         self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/vessel/pose', 10)
#         self.pub_velocity = self.create_publisher(TwistStamped, '/vessel/velocity', 10)
#             # after your existing publishers
#         self.pub_odometry = self.create_publisher(
#         Odometry,
#         '/er/odometry',
#         10
#     )


#         vp, hd = read_input()
#         self.vessel = Vessel(vp, hd, ros_flag=True)
#         self.vessel.reset()

#         self.ekf = EKF(sampling_rate=50, n_states=13, n_inp=1, logger=self.get_logger())
#         self.ekf.x = self.vessel.current_state.reshape(-1,1)
#         self.euler = None
#         self.R_imu = np.eye(9) * 0.0001
#         self.latest_imu_z = None
#         self.latest_uwb = None
#         self.uwb_updated= False
#         self.prev_uwb = None
#         self.quat=None

#         self.create_timer(1/50, self.timer_cb)

#     def imu_cb(self, msg: Imu):
#         quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
#         self.quat=quat
#         quat1=[quat[3],quat[0],quat[1],quat[2]]
        
#         self.euler = quat_to_eul(quat1)
#         yaw=self.euler[2]
#         self.get_logger().info(f"KF psi {yaw*180/np.pi}")
        
#         gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
#         accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
#         accel[2] -= 9.81
#         z = np.hstack([gyro, accel, self.euler]).reshape(-1,1)
#         self.latest_imu_z = z

#     def uwb_cb(self, msg: PoseWithCovarianceStamped):
#         pos = np.array([
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#             msg.pose.pose.position.z
#         ]).reshape(-1,1)
#         self.latest_uwb = pos
#         self.uwb_updated = True

#     # def actuator_cb(self, msg):
#     #     if "rudder" in msg.actuator_names:
#     #         index = msg.actuator_names.index("rudder")
#     #         rudder_deg = msg.actuator_values[index]
#     #         rudder_rad = np.deg2rad(rudder_deg)
#     #         self.vessel.current_state[12] = rudder_rad
#     # def actuator_cb(self, msg: Actuator):
#     #     self.get_logger().info("actuator_cb triggered")
#     # # find the rudder entry
#     #     try:
#     #         idx = msg.actuator_names.index('rudder')
#     #     except ValueError:
#     #         return  #no rudder in this message

#     #     # read the commanded angle (here 35.0) and convert to radians
#     #     angle_deg = msg.actuator_values[idx]
#     #     angle_rad = np.deg2rad(angle_deg)

#     #     # store it into your vessel state (state index 12)
#     #     self.vessel.current_state[12] = angle_rad
#     #     self.get_logger().info(f"rudder angle {angle_rad*180/np.pi}") 
#     def actuator_cb(self, msg: Actuator):
#         self.get_logger().info("actuator_cb triggered")
        
#         # Log the received message
#         self.get_logger().info(f"Received actuator_names: {msg.actuator_names}")
#         self.get_logger().info(f"Received actuator_values: {msg.actuator_values}")
        
#         try:
#             idx = msg.actuator_names.index('rudder')
#         except ValueError:
#             self.get_logger().warn("No 'rudder' entry in actuator_names")
#             return

#         angle_deg = msg.actuator_values[idx]
#         self.get_logger().info(f"Rudder angle (degrees): {angle_deg}")
        
#         angle_rad = np.deg2rad(angle_deg)
#         self.get_logger().info(f"Rudder angle (radians): {angle_rad}")
        
#         self.vessel.current_state[12] = angle_rad
#         self.get_logger().info(f"Stored rudder angle: {angle_rad * 180 / np.pi}")


#     def timer_cb(self):
#         self.vessel.step()
#         rudder = self.vessel.current_state[12]
#         u = np.array([rudder])
#         self.ekf.predict(u, self.vessel.vessel_ode)

#         # IMU correction
#         if self.latest_imu_z is not None:
#             def h_imu(x):
#                 rates = x[3:6]
#                 deriv = self.vessel.vessel_ode(0, x)
#                 accel = deriv[0:3]
#                 att = x[9:12]
#                 return np.hstack([rates, accel, att])
#             C_imu = self.ekf.jacobian(h_imu, self.ekf.x.flatten())
#             try:
#                 self.ekf.correct(self.latest_imu_z.flatten(), C_imu, self.R_imu, h_imu)
#             except np.linalg.LinAlgError as e:
#                 self.get_logger().error(f"IMU correction error: {e}")

#         # UWB correction
#         if self.latest_uwb is not None:
#             def h_uwb(x):

#                 return x[6:9]
#             C_uwb = np.zeros((3, self.ekf.n_states))
#             C_uwb[:, 6:9] = np.eye(3)
#             R_uwb = np.eye(3) *.05
#             self.ekf.correct(self.latest_uwb.flatten(), C_uwb, R_uwb, h_uwb)
#             self.ekf.x[6:9]=self.latest_uwb

#             # now detect no motion
#             if self.prev_uwb is not None:
#                 pos_threshold= 1e-5
#                 delta = self.latest_uwb - self.prev_uwb
#                 if np.linalg.norm(delta) <pos_threshold:  # threshold 
#                     # zero out surge/sway/heave velocities
#                     # self.ekf.x[0:3, 0] = 0.01
#                     # self.ekf.x[6:9]=self.latest_uwb 
#                     def h_vel(x): return x[0:3]
#                     C_vel = np.zeros((3,13)); C_vel[:,0:3] = np.eye(3)
#                     R_vel = np.eye(3) * 1e-4
#                     self.ekf.correct(np.zeros(3), C_vel, R_vel, h_vel)

#             # remember for next tick
#             self.prev_uwb = self.latest_uwb.copy()
#             self.uwb_updated = False
#         # Publish estimated pose
#         out = PoseWithCovarianceStamped()
#         out.header.stamp = self.get_clock().now().to_msg()
#         out.header.frame_id = 'world'
#         out.pose.pose.position.x = float(self.ekf.x[6,0])
#         out.pose.pose.position.y = float(self.ekf.x[7,0])
#         out.pose.pose.position.z = float(self.ekf.x[8,0])
#         self.pub_pose.publish(out)

#         # Publish body velocity 
#         u = self.ekf.x[0,0]
#         v = self.ekf.x[1,0]
#         w = self.ekf.x[2,0]
#         p = self.ekf.x[3,0]
#         q = self.ekf.x[4,0]
#         r = self.ekf.x[5,0]

#         vel_msg = TwistStamped()
#         vel_msg.header.stamp = self.get_clock().now().to_msg()
#         vel_msg.header.frame_id = 'vessel'
#         vel_msg.twist.linear.x = float(u)
#         vel_msg.twist.linear.y = float(v)
#         vel_msg.twist.linear.z = float(w)
#         vel_msg.twist.angular.x = float(p)
#         vel_msg.twist.angular.y = float(q)
#         vel_msg.twist.angular.z = float(r)
#         self.pub_velocity.publish(vel_msg)
#         odom=Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = 'world'
#         odom.child_frame_id = 'vessel'
#         odom.pose.pose.position.x = float(self.ekf.x[6,0])
#         odom.pose.pose.position.y = float(self.ekf.x[7,0])
#         if self.quat is not None:
#             odom.pose.pose.orientation = Quaternion(x=self.quat[1], y=self.quat[2], z=self.quat[3], w=self.quat[0])
#         #odom.pose.pose.position.z = float(self.ekf.x[8,0])
#         yaw= self.ekf.x[11,0]
#         odom.twist.twist.angular.z = yaw
#         self.pub_odometry.publish(odom)


#        # self.get_logger().info(
#             #f"Surge: {self.ekf.x[0,0]} m/s | Sway: {self.ekf.x[1,0]} m/s | Heave: {w:.3f} m/s | yaw :{yaw*57.3} | position x:{self.ekf.x[6,0]}  | position y:{self.ekf.x[7,0]} | position z:{self.ekf.x[8,0]}  "
#        # )

# def main(args=None):
#     rclpy.init(args=args)
#     node = SimpleEKFNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Quaternion
from interfaces.msg import Actuator
import numpy as np
import sys

sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.read_input import read_input
from ros2_ws.src.student.tut_03.class_vessel import Vessel
from ros2_ws.src.student.tut_03.simulate import simulate
from ros2_ws.src.student.tut_03.module_kinematics import (
    eul_to_quat,
    quat_to_eul,
    ssa,
    clip
)
from nav_msgs.msg import Odometry

class EKF:
    def __init__(self, sampling_rate, n_states=13, n_inp=1, logger=None):
        self.dt = 1.0 / sampling_rate
        self.n_states = n_states
        self.n_inp = n_inp
        self.x = np.zeros((n_states, 1))
        self.P = np.eye(n_states) * 0.1
        self.E = np.eye(n_states)
        # process noise
        self.Q = np.diag([5, 5, 5] + [5] * (n_states-3))
        self.logger = logger

    def jacobian(self, fun, x0):
        x0 = x0.flatten()
        f0 = fun(x0)
        m, n = f0.size, x0.size
        J = np.zeros((m, n))
        eps = 1e-5
        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps
            f1 = fun(x0 + dx)
            f2 = fun(x0 - dx)
            J[:, i] = (f1 - f2) / (2 * eps)
        return J

    def predict(self, u, plant_model):
        x0 = self.x.flatten()
        dt = self.dt
        f = lambda t, x: plant_model(t, x, u)
        k1 = plant_model(0, x0)
        k2 = plant_model(dt/2, x0 + dt/2 * k1)
        k3 = plant_model(dt/2, x0 + dt/2 * k2)
        k4 = plant_model(dt, x0 + dt * k3)
        x_pred = x0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        F = self.jacobian(lambda x: f(0, x), x_pred)
        try:
            P_pred = F @ self.P @ F.T + self.E @ self.Q @ self.E.T
        except np.linalg.LinAlgError as e:
            if self.logger:
                self.logger.error(f"Prediction failed: {e}")
            return
        self.x = x_pred.reshape(-1, 1)
        self.P = P_pred

    def correct(self, y, C, R, meas_model):
        I = np.eye(self.n_states)
        R = R + np.eye(R.shape[0])  # stability
        S = C @ self.P @ C.T + R
        try:
            U, s, Vh = np.linalg.svd(S)
            S_inv = Vh.T @ np.diag(1.0 / s) @ U.T
        except np.linalg.LinAlgError as e:
            if self.logger:
                self.logger.error(f"Correction failed: {e}")
            return
        K = self.P @ C.T @ S_inv
        y = y.reshape(-1,1)
        h = meas_model(self.x.flatten()).reshape(-1,1)
        innov = y - h
        dx = K @ innov
        for i in range(3):
            dx[i,0] = clip(dx[i,0], 1.0)
        self.x += dx
        self.P = (I - K @ C) @ self.P @ (I - K @ C).T + K @ R @ K.T
        self.P = (self.P + self.P.T) / 2
        # clamp velocities
        self.x[0:3] = np.clip(self.x[0:3], -0.5, 0.5)
        self.x[3:6] = np.clip(self.x[3:6], -0.1, 0.1)
        # wrap attitudes
        for i in [9,10,11]:
            self.x[i,0] = ssa(self.x[i,0])

class SimpleEKFNode(Node):
    def __init__(self):
        super().__init__('simple_ekf_node')
        # subscriptions
        self.create_subscription(Imu, '/sookshma_00/imu/data', self.imu_cb, 10)
        self.create_subscription(
            Actuator, '/sookshma_00/actuator_cmd', self.actuator_cb, 10
        )
        # odometry publisher
        self.pub_odometry = self.create_publisher(
            Odometry, '/er/odometry', 10
        )
        # vessel model
        vp, hd = read_input()
        self.vessel = Vessel(vp, hd, ros_flag=True)
        self.vessel.reset()
        # EKF init
        self.ekf = EKF(sampling_rate=50, logger=self.get_logger())
        self.ekf.x = self.vessel.current_state.reshape(-1,1)
        # buffers
        self.euler = None
        self.quat = None
        self.latest_imu_z = None
        self.latest_uwb = None
        self.prev_uwb = None
        # covariances
        self.R_imu = np.eye(9) * 1e-4
        # timer
        self.create_timer(1/50, self.timer_cb)

    def imu_cb(self, msg: Imu):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        # store for odom orientation
        self.quat = [q[3], q[0], q[1], q[2]]
        self.euler = quat_to_eul(self.quat)
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        accel[2] -= 9.81
        z = np.hstack([gyro, accel, self.euler]).reshape(-1,1)
        self.latest_imu_z = z

    def actuator_cb(self, msg: Actuator):
        self.get_logger().info("actuator_cb triggered")
        try:
            idx = msg.actuator_names.index('rudder')
        except ValueError:
            self.get_logger().warn("No 'rudder' in actuator_names")
            return
        angle_rad = np.deg2rad(msg.actuator_values[idx])
        self.vessel.current_state[12] = angle_rad

    def timer_cb(self):
        # simulate vessel
        self.vessel.step()
        u = np.array([self.vessel.current_state[12]])
        # EKF predict
        self.ekf.predict(u, self.vessel.vessel_ode)
        # IMU update
        if self.latest_imu_z is not None:
            def h_imu(x):
                rates = x[3:6]
                deriv = self.vessel.vessel_ode(0, x)
                accel = deriv[0:3]
                att = x[9:12]
                return np.hstack([rates, accel, att])
            C_imu = self.ekf.jacobian(h_imu, self.ekf.x.flatten())
            self.ekf.correct(self.latest_imu_z.flatten(), C_imu, self.R_imu, h_imu)
        # UWB update
        if self.latest_uwb is not None:
            def h_uwb(x): return x[6:9]
            C_uwb = np.zeros((3, self.ekf.n_states))
            C_uwb[:,6:9] = np.eye(3)
            R_uwb = np.eye(3) * 0.05
            self.ekf.correct(self.latest_uwb.flatten(), C_uwb, R_uwb, h_uwb)
            # detect static
            if self.prev_uwb is not None:
                if np.linalg.norm(self.latest_uwb - self.prev_uwb) < 1e-5:
                    def h_vel(x): return x[0:3]
                    C_vel = np.zeros((3,13)); C_vel[:,0:3] = np.eye(3)
                    R_vel = np.eye(3) * 1e-4
                    self.ekf.correct(np.zeros(3), C_vel, R_vel, h_vel)
            self.prev_uwb = self.latest_uwb.copy()
        # publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'vessel'
        odom.pose.pose.position.x = float(self.ekf.x[6,0])
        odom.pose.pose.position.y = float(self.ekf.x[7,0])
        if self.quat:
            odom.pose.pose.orientation = Quaternion(
                x=self.quat[1], y=self.quat[2], z=self.quat[3], w=self.quat[0]
            )
        odom.twist.twist.angular.z = float(self.ekf.x[11,0])
        self.pub_odometry.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleEKFNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

