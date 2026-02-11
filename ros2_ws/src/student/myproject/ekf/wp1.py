
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix, Imu
# from std_msgs.msg import String
# from geometry_msgs.msg import Vector3
# #from interfaces.msg import EstimatedState
# from nav_msgs.msg import Odometry
# from interfaces.msg import Actuator
# from std_msgs.msg import String
# import numpy as np
# import math
# from scipy.spatial.transform import Rotation
# from math import sin, cos
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
# class ONRTBoat(Node):
#     def __init__(self):
#         super().__init__('onrt_boat_controller')

#         # Publishers & Subscribers
#         self.actuator_publisher = self.create_publisher(
#             Actuator, '/sookshma_00/actuator_cmd', 10)
#         # self.create_subscription(
#         #     PoseWithCovarianceStamped, '/vessel/pose', self.pose_cb, 10)
#         self.create_subscription(
#             Odometry, '/er/odometry', self.pose_cb, 10)
#         # Subscribe directly to EKF-published yaw
#         # self.create_subscription(
#         #     Float64, '/vessel/yaw', self.yaw_cb, 10)

#         # State variables
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.r=0

#         # LOS control variables
#         self.xpe = 0.0
#         self.ype = 0.0
#         self.yp_int = 0.0
#         self.psi_des = 0.0
#         self.previous_angle = 0.0
#         self.last_time = self.get_clock().now()

#         # PID/LOS gains
#         self.kp_angular = -.5
#         self.kd_angular = .2
#         self.k_los = 0.1
#         self.ship_length = 0.4
#         self.lookahead_distance = 1.5

#         # Waypoint path
#         self.path = [[12, 7],[13,21],[7,19],[13,24]]
#         self.current_goal_index = 0

#         # Constants
#         self.RPM = 450.0
        
#         self.MAX_RUDDER_DEG = 35.0

#         # Main control timer (10 Hz)
#         self.create_timer(0.1, self.control_loop)

#     # def yaw_cb(self, msg: Float64):
#     #     # Direct yaw from EKF
#     #     self.yaw = msg.data
#     #     self.get_logger().debug(f"Received EKF yaw: {self.yaw:.1f}°")

#     def pose_cb(self, msg: Odometry):
#         # Update position
#         p = msg.pose.pose.position
#         self.x, self.y = p.x, p.y
#         self.yaw= msg.pose.pose.orientation
#         self.get_logger().debug(
#             f"Pose received: x={self.x:.2f}, y={self.y:.2f}"
#         )

#     # def vel_cb(self, msg: TwistStamped):
#     #     # Log body-frame velocities for debugging
#     #     u = msg.twist.linear.x
#     #     v = msg.twist.linear.y
#     #     self.get_logger().info(
#     #         f"[Wp {self.current_goal_index}] x={self.x:.2f}, y={self.y:.2f}, "
#     #         f"yaw={(self.yaw)*57.3:.1f}°, u={u:.2f}, v={v:.2f}"
#     #     )
#     def clip(self,value, threshold):
#         """Clip a value to +/- threshold.
        
#         Args:
#             value (float): Input value
#             threshold (float): Maximum absolute value
        
#         Returns:
#             float: Clipped value
#         """
#         if np.abs(value) > threshold:
#             return np.sign(value) * threshold
#         else:
#             return value
#     def quaternion_to_euler(self,quat):
#         """
#         Converts a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

#         Args:
#             x (float): Quaternion x component.
#             y (float): Quaternion y component.
#             z (float): Quaternion z component.
#             w (float): Quaternion w component.

#         Returns:
#             tuple: (roll, pitch, yaw) in radians.
#         """
#         x, y, z, w = quat
#         # Roll (x-axis rotation)
#         t0 = 2.0 * (w * x + y * z)
#         t1 = 1.0 - 2.0 * (x * x + y * y)
#         roll = np.arctan2(t0, t1)

#         # Pitch (y-axis rotation)
#         t2 = 2.0 * (w * y - z * x)
#         t2 = np.clip(t2, -1.0, 1.0)  # Clamp to avoid numerical errors
#         pitch = np.arcsin(t2)

#         # Yaw (z-axis rotation)
#         t3 = 2.0 * (w * z + x * y)
#         t4 = 1.0 - 2.0 * (y * y + z * z)
#         yaw = np.arctan2(t3, t4)

#         return roll, pitch, yaw
#     def pose_cb(self, msg: Odometry):
#     # Update position
#         p = msg.pose.pose.position
#         self.x, self.y = p.x, p.y
#         self.r= msg.twist.twist.angular.z
#         # Extract yaw from quaternion
#         q = msg.pose.pose.orientation
#         quat_list = [q.w, q.x, q.y, q.z]
#         _, _, self.yaw = self.quaternion_to_euler(quat_list)

#         # Log position and heading in one string
#         self.get_logger().info(
#             f"Pose received: x={self.x:.2f}, y={self.y:.2f}, "
#             f"yaw={math.degrees(self.yaw):.2f}°"
#         )

#     def control_loop(self):
#         # If reached final waypoint, stop
#         if self.current_goal_index >= len(self.path) - 1:
#             self.stop()
#             return

#         # Get current and next waypoints
#         cg = self.path[self.current_goal_index]
#         ng = self.path[self.current_goal_index + 1]

#         # Compute commands
#         rudder_deg = self.calculate_rudder(cg, ng)
#         rudder_deg =self.clip(rudder_deg,35) #max(min(rudder_deg, self.MAX_RUDDER_DEG), -self.MAX_RUDDER_DEG)
#         self.publish_actuator_command(self.RPM, rudder_deg)

#         # Advance waypoint if close enough to next goal
#         dx = self.x - ng[0]
#         dy = self.y - ng[1]
#         dist = math.hypot(dx, dy)
#         if dist < 1.0:
#             self.current_goal_index += 1
#     def ssa(self,ang, deg=False):
#         """Convert angle to smallest signed angle.
        
#         Args:
#             ang (float): Input angle
#             deg (bool, optional): If True, angle is in degrees. Defaults to False.
        
#         Returns:
#             float: Smallest signed angle equivalent to input
#         """
#         if deg:
#             ang = (ang + 180) % (360.0) - 180.0
#         else:
#             ang = (ang + np.pi) % (2 * np.pi) - np.pi
#         return ang

#     def calculate_rudder(self, current_goal, next_goal):
#         # Path angle
#         dx = next_goal[0] - current_goal[0]
#         dy = next_goal[1] - current_goal[1]
#         path_angle = math.atan2(dy, dx)

#         # Cross-track & along-track
#         ex = self.x - current_goal[0]
#         ey = self.y - current_goal[1]
#         self.ype = -ex*math.sin(path_angle) + ey*math.cos(path_angle)
#         self.xpe =  ex*math.cos(path_angle) + ey*math.sin(path_angle)

#         # LOS desired heading
#         kp = 1.0/self.lookahead_distance

#         ki = kp*self.k_los
#         ki=0
#         self.psi_des = path_angle - math.atan(kp*self.ype + ki*self.yp_int)

#         # Integrate cross-track error
#         now = self.get_clock().now()
#         dt = (now - self.last_time).nanoseconds/1e9
#         self.yp_int += self.lookahead_distance*self.ype * dt
#         self.last_time = now
#         dgoal=np.sqrt((self.x-next_goal[0])**2 + (self.y-next_goal[1])**2)

#         # Heading error and rate
#         err_ssa = self.ssa(self.psi_des- self.yaw )
#         rudder = self.kp_angular*err_ssa-self.kd_angular*self.r
#         self.get_logger().info(f"RUDDER: {rudder:.2f}°")
#         #err=self.kd_angular*self.r
#         # derr = (err - self.previous_angle)/dt if dt>1e-6 else 0.0
#         # self.previous_angle = err
#         self.get_logger().info(
#     f"Heading error: {self.ype:.2f}°, "
#     f"Desired heading: {math.degrees(self.psi_des):.2f}°, "
#     f"Cross-track error: {self.ype:.2f} m, "
#     f"Along-track error: {self.xpe:.2f} m, "
#     f"Position: ({self.x:.2f}, {self.y:.2f}), "
#     f"curret_heading: {math.degrees(self.yaw):.2f}°, "
#     f"Path angle: {math.degrees(path_angle):.2f}°, "
#     f"Current goal: ({current_goal[0]:.2f}, {current_goal[1]:.2f}), "
#     f"Next goal: ({next_goal[0]:.2f}, {next_goal[1]:.2f})"
#     f"distance_goal:({dgoal:.2f}), "
#     f"Eror_ssa: {np.rad2deg(err_ssa):.2f}°"
#     f"kdtimesr: {(self.kd_angular*self.r):.2f}°"
# )


        
#         # PD control
#         # rudder = -(self.kp_angular*err
#         return math.degrees(rudder)

#     def publish_actuator_command(self, rpm, rudder_deg):
#         m = Actuator()
#         m.header.stamp = self.get_clock().now().to_msg()
#         m.actuator_names = ['rudder','propeller']
#         m.actuator_values = [float(rudder_deg), float(rpm)]
#         self.actuator_publisher.publish(m)
#         self.get_logger().info(
#             f"Cmd: propeller={rpm:.1f}, rudder={rudder_deg:.1f}°"
#         )

#     def stop(self):
#         # Zero thrust and rudder
#         self.publish_actuator_command(0.0, 0.0)
#         self.get_logger().info("Completed all waypoints, vessel stopped.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ONRTBoat()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
#from interfaces.msg import EstimatedState
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from std_msgs.msg import String
import numpy as np
import math
from scipy.spatial.transform import Rotation
from math import sin, cos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
class ONRTBoat(Node):
    def __init__(self):
        super().__init__('onrt_boat_controller')

        # Publishers & Subscribers
        self.actuator_publisher = self.create_publisher(
            Actuator, '/sookshma_00/actuator_cmd', 10)
        # self.create_subscription(
        #     PoseWithCovarianceStamped, '/vessel/pose', self.pose_cb, 10)
        self.create_subscription(
            Odometry, '/sookshma_00/odometry', self.pose_cb, 10)
        # Subscribe directly to EKF-published yaw
        # self.create_subscription(
        #     Float64, '/vessel/yaw', self.yaw_cb, 10)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.r=0

        # LOS control variables
        self.xpe = 0.0
        self.ype = 0.0
        self.yp_int = 0.0
        self.psi_des = 0.0
        self.previous_angle = 0.0
        self.last_time = self.get_clock().now()

        # PID/LOS gains
        self.kp_angular = -.5
        self.kd_angular = .25
        self.k_los = 0.1
        self.ship_length = 0.4
        self.lookahead_distance = 3

        # Waypoint path
        self.path = [[6, 6], [15, 6], [15,20],[6,20],[6,6]]
        self.current_goal_index = 0

        # Constants
        self.RPM = 450.0
        
        self.MAX_RUDDER_DEG = 35.0

        # Main control timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

    # def yaw_cb(self, msg: Float64):
    #     # Direct yaw from EKF
    #     self.yaw = msg.data
    #     self.get_logger().debug(f"Received EKF yaw: {self.yaw:.1f}°")

    def pose_cb(self, msg: Odometry):
        # Update position
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw= msg.pose.pose.orientation
        self.get_logger().debug(
            f"Pose received: x={self.x:.2f}, y={self.y:.2f}"
        )

    # def vel_cb(self, msg: TwistStamped):
    #     # Log body-frame velocities for debugging
    #     u = msg.twist.linear.x
    #     v = msg.twist.linear.y
    #     self.get_logger().info(
    #         f"[Wp {self.current_goal_index}] x={self.x:.2f}, y={self.y:.2f}, "
    #         f"yaw={(self.yaw)*57.3:.1f}°, u={u:.2f}, v={v:.2f}"
    #     )
    def clip(self,value, threshold):
        """Clip a value to +/- threshold.
        
        Args:
            value (float): Input value
            threshold (float): Maximum absolute value
        
        Returns:
            float: Clipped value
        """
        if np.abs(value) > threshold:
            return np.sign(value) * threshold
        else:
            return value
    def quaternion_to_euler(self,quat):
        """
        Converts a quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).

        Args:
            x (float): Quaternion x component.
            y (float): Quaternion y component.
            z (float): Quaternion z component.
            w (float): Quaternion w component.

        Returns:
            tuple: (roll, pitch, yaw) in radians.
        """
        x, y, z, w = quat
        # Roll (x-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # Clamp to avoid numerical errors
        pitch = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw
    def pose_cb(self, msg: Odometry):
    # Update position
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.r= msg.twist.twist.angular.z
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        quat_list = [q.x, q.y, q.z, q.w]
        _, _, self.yaw = self.quaternion_to_euler(quat_list)

        # Log position and heading in one string
        self.get_logger().info(
            f"Pose received: x={self.x:.2f}, y={self.y:.2f}, "
            f"yaw={math.degrees(self.yaw):.2f}°"
        )

    def control_loop(self):
        # If reached final waypoint, stop
        if self.current_goal_index >= len(self.path) - 1:
            self.stop()
            return

        # Get current and next waypoints
        cg = self.path[self.current_goal_index]
        ng = self.path[self.current_goal_index + 1]

        # Compute commands
        rudder_deg = self.calculate_rudder(cg, ng)
        rudder_deg =self.clip(rudder_deg,35) #max(min(rudder_deg, self.MAX_RUDDER_DEG), -self.MAX_RUDDER_DEG)
        self.publish_actuator_command(self.RPM, rudder_deg)

        # Advance waypoint if close enough to next goal
        dx = self.x - ng[0]
        dy = self.y - ng[1]
        dist = math.hypot(dx, dy)
        if dist < 1.0:
            self.current_goal_index += 1
    def ssa(self,ang, deg=False):
        """Convert angle to smallest signed angle.
        
        Args:
            ang (float): Input angle
            deg (bool, optional): If True, angle is in degrees. Defaults to False.
        
        Returns:
            float: Smallest signed angle equivalent to input
        """
        if deg:
            ang = (ang + 180) % (360.0) - 180.0
        else:
            ang = (ang + np.pi) % (2 * np.pi) - np.pi
        return ang

    def calculate_rudder(self, current_goal, next_goal):
        # Path angle
        dx = next_goal[0] - current_goal[0]
        dy = next_goal[1] - current_goal[1]
        path_angle = math.atan2(dy, dx)

        # Cross-track & along-track
        ex = self.x - current_goal[0]
        ey = self.y - current_goal[1]
        self.ype = -ex*math.sin(path_angle) + ey*math.cos(path_angle)
        self.xpe =  ex*math.cos(path_angle) + ey*math.sin(path_angle)

        # LOS desired heading
        kp = 1.0/self.lookahead_distance

        ki = kp*self.k_los
        #ki=0
        self.psi_des = path_angle - math.atan(kp*self.ype + ki*self.yp_int)

        # Integrate cross-track error
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds/1e9
        self.yp_int += self.lookahead_distance*self.ype * dt
        self.last_time = now
        dgoal=np.sqrt((self.x-next_goal[0])**2 + (self.y-next_goal[1])**2)

        # Heading error and rate
        err_ssa = self.ssa(self.psi_des- self.yaw )
        rudder = self.kp_angular*err_ssa-self.kd_angular*self.r
        self.get_logger().info(f"RUDDER: {rudder:.2f}°")
        #err=self.kd_angular*self.r
        # derr = (err - self.previous_angle)/dt if dt>1e-6 else 0.0
        # self.previous_angle = err
        self.get_logger().info(
    f"Heading error: {self.ype:.2f}°, "
    f"Desired heading: {math.degrees(self.psi_des):.2f}°, "
    f"Cross-track error: {self.ype:.2f} m, "
    f"Along-track error: {self.xpe:.2f} m, "
    f"Position: ({self.x:.2f}, {self.y:.2f}), "
    f"curret_heading: {math.degrees(self.yaw):.2f}°, "
    f"Path angle: {math.degrees(path_angle):.2f}°, "
    f"Current goal: ({current_goal[0]:.2f}, {current_goal[1]:.2f}), "
    f"Next goal: ({next_goal[0]:.2f}, {next_goal[1]:.2f})"
    f"distance_goal:({dgoal:.2f}), "
    f"Eror_ssa: {np.rad2deg(err_ssa):.2f}°"
    f"kdtimesr: {(self.kd_angular*self.r):.2f}°"
)


        
        # PD control
        # rudder = -(self.kp_angular*err
        return math.degrees(rudder)

    def publish_actuator_command(self, rpm, rudder_deg):
        m = Actuator()
        m.header.stamp = self.get_clock().now().to_msg()
        m.actuator_names = ['rudder','propeller']
        m.actuator_values = [float(rudder_deg), float(rpm)]
        self.actuator_publisher.publish(m)
        self.get_logger().info(
            f"Cmd: propeller={rpm:.1f}, rudder={rudder_deg:.1f}°"
        )

    def stop(self):
        # Zero thrust and rudder
        self.publish_actuator_command(0.0, 0.0)
        self.get_logger().info("Completed all waypoints, vessel stopped.")


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