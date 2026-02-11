
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
from geometry_msgs.msg import Point, Vector3, Pose2D, Quaternion, Twist, PoseWithCovarianceStamped
class ONRTBoat(Node):
    def __init__(self):
        super().__init__('onrt_boat_controller')

        # Publishers & Subscribers
        self.actuator_publisher = self.create_publisher(
            Actuator, '/kurma_00/actuator_cmd', 10)
        # self.create_subscription(
        #     PoseWithCovarianceStamped, '/vessel/pose', self.pose_cb, 10)
        self.create_subscription(
            Odometry, '/kurma_00/odometry', self.pose_cb, 10)
        # Subscribe directly to EKF-published yaw
        # self.create_subscription(
        #     Float64, '/vessel/yaw', self.yaw_cb, 10)
        self.create_subscription(
            Pose2D, '/pursuer1/pose', self.pursuer1_pose_callback, 10)
        self.create_subscription(
            Pose2D, '/pursuer2/pose', self.pursuer2_pose_callback, 10)
        self.create_subscription(
            Pose2D, '/pursuer3/pose', self.pursuer3_pose_callback, 10)
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
        self.kp_angular = -1.5
        self.kd_angular = .5
        self.k_los = 0.5
        self.ship_length = 0.4
        self.lookahead_distance = 3

        # Waypoint path
        self.path = [[6,6],[15,6],[15,20],[6,20],[6,6]]
        self.current_goal_index = 0

        # Constants
        self.RPM = 450.0
        
        self.MAX_RUDDER_DEG = 35.0

        # Main control timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

    
    def pose_cb(self, msg: Odometry):
        # Update position
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw= msg.pose.pose.orientation
        self.get_logger().debug(
            f"Pose received: x={self.x:.2f}, y={self.y:.2f}"
        )


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
        if dist < 3.0:
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
            """
            Modified rudder calculation using APF-based evasion + goal-seeking
            """
            # Control gains
            K_att = 0.005
            K_rep_base = 400000.0
            d0 = 800.0
            eps = 1e-6
            max_tc = 30.0
            
            # Current position and goal
            pos = np.array([self.x, self.y])
            goal = np.array(next_goal)
            
            # Get closest pursuer distance (assuming pursuers publish their positions)
            pursuer_positions = self.get_pursuer_positions()
            closest_pursuer_dist = float('inf')
            if pursuer_positions:
                closest_pursuer_dist = min(np.linalg.norm(p - pos) for p in pursuer_positions)
            
            # Calculate attractive force with weighted attraction
            attraction_weight = np.clip(closest_pursuer_dist / (2*d0), 0.1, 1.0)
            F_att = K_att * attraction_weight * (goal - pos)
            
            # Calculate repulsive forces from pursuers
            F_rep = np.zeros(2)
            for p_pos in pursuer_positions:
                diff = pos - p_pos
                dist = np.linalg.norm(diff)
                if dist < eps:
                    continue
                    
                # Simplified time-to-collision (without velocity)
                t_c = min(dist/1.0, max_tc)  # Assume unit closing speed
                
                # Exponential scaling of repulsion
                K_rep = K_rep_base * np.exp(-dist/d0) / (1.0 + 0.5*t_c)
                
                if dist < d0:
                    rep_mag = K_rep * (1.0/dist - 1.0/d0) / (dist**1.5)
                    F_rep += rep_mag * (diff / dist)
            
            # Combine forces
            F_total = F_att + F_rep
            
            # Calculate desired heading
            if np.linalg.norm(F_total) < eps:
                # If no clear guidance, maintain current heading
                psi_des = self.yaw
            else:
                psi_des = np.arctan2(F_total[1], F_total[0])
            
            # PD control with heading error
            err_ssa = self.ssa(psi_des - self.yaw)
            
            # Add emergency turn when pursuers are very close
            emergency_turn = 0.0
            if closest_pursuer_dist < d0/2:
                emergency_turn = np.sign(err_ssa) * 0.5 * np.radians(self.MAX_RUDDER_DEG)
            
            rudder = self.kp_angular * err_ssa - self.kd_angular * self.r + emergency_turn
            
            # Log debug info
            self.get_logger().info(
                f"Position: ({self.x:.2f}, {self.y:.2f}), "
                f"Current heading: {math.degrees(self.yaw):.2f}°, "
                f"Desired heading: {math.degrees(psi_des):.2f}°, "
                f"Closest pursuer: {closest_pursuer_dist:.2f}m, "
                f"Emergency turn: {math.degrees(emergency_turn):.2f}°"
            )
            
            return math.degrees(rudder)

    def get_pursuer_positions(self):
            """
            Get latest positions of pursuers from ROS topics
            Returns list of numpy arrays with pursuer positions
            """
            positions = []
            # Add pursuer positions from subscribed topics
            if hasattr(self, 'pursuer1_pose'):
                positions.append(np.array([self.pursuer1_pose.x, self.pursuer1_pose.y]))
            if hasattr(self, 'pursuer2_pose'):
                positions.append(np.array([self.pursuer2_pose.x, self.pursuer2_pose.y]))
            if hasattr(self, 'pursuer3_pose'):
                positions.append(np.array([self.pursuer3_pose.x, self.pursuer3_pose.y]))
            return positions

        # Add subscribers for pursuer positions
    # def __init__(self):
    #         # ... existing init code ...
            
    #         # Add subscribers for pursuer positions
    #         self.create_subscription(
    #             Pose2D, '/pursuer1/pose', self.pursuer1_pose_callback, 10)
    #         self.create_subscription(
    #             Pose2D, '/pursuer2/pose', self.pursuer2_pose_callback, 10)
    #         self.create_subscription(
    #             Pose2D, '/pursuer3/pose', self.pursuer3_pose_callback, 10)

        # Add callback handlers for pursuer positions
    def pursuer1_pose_callback(self, msg):
            self.pursuer1_pose = msg

    def pursuer2_pose_callback(self, msg):
            self.pursuer2_pose = msg

    def pursuer3_pose_callback(self, msg):
            self.pursuer3_pose = msg 


                
                

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