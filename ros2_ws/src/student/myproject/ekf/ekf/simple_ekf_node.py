
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
from scipy.spatial.transform import Rotation
from math import sin, cos
from std_msgs.msg import Float64

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped

#import pyproj


def quaternion_to_euler(quat):
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


class SimpleEKFNode(Node):
    def __init__(self):
        super().__init__('simple_ekf_node')
        
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 1000
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.R_uwb = np.diag([1.0, 1.0])
        self.R_imu = np.diag([0.1, 0.1])
        self.last_time = None
        self.rudder = 0.0
        self.propeller = 0.0
        self.time_stamp = None
        self.status = None
        self.quat = None    
        
        # Origin for uwb conversion
        self.origin = None
        
        
        self.uwb_sub = self.create_subscription(
            Vector3,
            '/sookshma_01/uwb',
            self.uwb_callback,
            10)
            
        self.imu_sub = self.create_subscription(
            Imu,
            '/sookshma_01/imu/data',
            self.imu_callback,
            10)
            
        self.actuator_sub = self.create_subscription(
            String,
            '/sookshma_01/actuator_feedback',
            self.actuator_callback,
            10)
        # self.status_sub = self.create_subscription(
        #     String,  # Subscribe to String messages
        #     '/sookshma/current_status',  # Updated topic
        #     self.status_callback,  # New callback method
        #     10)

        # self.actuator_sub =  self.create_subscription(
        #     String,  # Change to String since we are subscribing to a String message
        #     '/sookshma/thrust_command',  # Updated topic
        #     self.actuator_callback,  # New callback method
        #     10)


                # QoS Profile: Best Effort, Keep only the latest message
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # self.state_pub = self.create_publisher(
        #     EstimatedState,
        #     '~/state_estimate',  # debug topic 
        #     10
        #     # qos_profile
        # )
        self.odom_pub = self.create_publisher(Odometry,
         '/er/odometry', 10)
        self.yaw_pub = self.create_publisher(Float64, '/er/yaw', 10)
                # Create a timer for publishing state at 10Hz (0.1 seconds)
        self.publish_rate = 10.0  # Hz
        self.publish_timer = self.create_timer(
            1.0/self.publish_rate,  # seconds
            self.timer_callback
        )
        
        self.state_updated = False

    def timer_callback(self):
        """Callback function for the timer that publishes state at a fixed rate."""
        if self.state_updated:
            self.publish_state()
            self.state_updated = False

    
   #     return x, y

    def predict(self, dt):
        F = np.eye(6)
        F[3,0] = dt * cos(self.x[5,0]) # u contr in X
        F[3,1] = -dt * sin(self.x[5,0]) # v in X
        F[4,0] = dt * sin(self.x[5,0]) # u in Y
        F[4,1] = dt * cos(self.x[5,0]) # v in Y 
        # self.get_logger().info(f"x : {F}")
        
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update_uwb(self, x, y):
        # if self.origin is None:
        #     self.origin_x = 0
        #     self.origin_y = 0
        #     return
        
        # Transform measurements to be relative to the stored origin
        # x_relative = x - self.origin_x
        # y_relative = y - self.origin_y
        
        measured_pos = np.array([[x], [y]])
        
        H = np.zeros((2, 6))
        H[0,3] = 1.0
        H[1,4] = 1.0
        
        self.update(measured_pos, H, self.R_uwb)

    def update_imu(self, r, psi):
        measured_imu = np.array([[r], [psi]])
        
        H = np.zeros((2, 6))
        H[0,2] = 1.0
        H[1,5] = 1.0
        
        self.update(measured_imu, H, self.R_imu)

    def update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def uwb_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        

        # x_ned, y_ned, z = self.eul_to_rotm([
        #          #0, 0, 0
        #         np.pi, 0, -np.pi/2
        #     ])@np.array([msg.x, msg.y, 0])

        if self.last_time is not None:
            dt = current_time - self.last_time
        
            self.predict(dt)
            
        self.update_uwb(msg.x, msg.y)
        # self.update_uwb(msg.x, msg.y)
        # self.get_logger().info(f"x {x_ned} y {y_ned}")
        # self.get_logger().info(f"x {msg.x} y {msg.y}")
        self.last_time = current_time
        self.state_updated = True 
        # self.publish_state()

    def imu_callback(self, msg):
        self.time_stamp = msg.header.stamp
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            
            self.predict(dt)
        
        quat = [msg.orientation.x, msg.orientation.y, 
                msg.orientation.z, msg.orientation.w]
        self.quat = quat
        # euler = Rotation.from_quat(quat).as_euler('zyx')
        euler = quaternion_to_euler(quat)
        psi = euler[2]
        #self.get_logger().info(f"KF psi {psi*180/np.pi}")
        self.get_logger().info(f"KF psi {psi*180/np.pi:.2f}°, surge = {self.x[0, 0]:.3f} m/s, sway = {self.x[1, 0]:.3f} m/s")
        r = msg.angular_velocity.z
        
        self.update_imu(r, psi)
        self.last_time = current_time
        self.state_updated = True 
        # self.publish_state()


    def status_callback(self, msg):

        self.status = int(msg.data)

    def actuator_callback(self, msg):
        # if self.status==1:
            # Parse the thrust_command_msg.data to extract propeller and rudder values
        data_parts = msg.data.split(',')
        self.propeller = float(data_parts[0].split(':')[1])  # Extract propeller value
        self.rudder = float(data_parts[1].split(':')[1])      # Extract rudder value
        # elif self.status==0:
            # If no valid data, set propeller and rudder to zero
            # self.propeller = -100.0
            # self.rudder = -100.0


        # data_parts = msg.data.split(',')
        # self.propeller = float(data_parts[0].split(':')[1])  # Extract propeller value
        # self.rudder = float(data_parts[1].split(':')[1])      # Extract rudder value

        # self.rudder = msg.rudder 
        # self.propeller = msg.propeller


    def eul_to_rotm(self, eul, order='ZYX', deg=False):
        if deg:
            eul = np.radians(eul)
        
        phi, theta, psi = eul  # Roll, Pitch, Yaw

        # Rotation about Z-axis (Yaw)
        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])

        # Rotation about Y-axis (Pitch)
        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

        # Rotation about X-axis (Roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])

        # Combined rotation matrix: ZYX order
        rotm = R_z @ R_y @ R_x

        return rotm

    def publish_odom(self):
        position = [ float(self.x[3, 0]),  float(self.x[4, 0])]
        quat = self.quat 
        odom_msg = Odometry() 
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'NED'
        odom_msg.pose.pose.position = Point(x=float(position[0]), y= float(position[1]), z=0.0)
        # quat = self.eul_to_quat([0.0, 0.0, float(self.x[5, 0])])
        if self.quat:odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        odom_msg.twist.twist.linear.x = float(self.x[0, 0])  # Surge (u)
        odom_msg.twist.twist.linear.y = float(self.x[1, 0])  # Sway (v)
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(self.x[2, 0])
        self.odom_pub.publish(odom_msg)

    def publish_yaw(self):
        if self.quat is not None:
            _, _, yaw = quaternion_to_euler(self.quat)
            yaw_msg = Float64()
            yaw_msg.data = np.pi*yaw/180 # In degrees
            self.yaw_pub.publish(yaw_msg)
    def publish_state(self):
        # msg = EstimatedState()
        
        # msg.header.frame_id = 'kf'
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.u = float(self.x[0, 0])
        # msg.v = float(self.x[1, 0])
        # msg.r = float(self.x[2, 0])
        # msg.x = float(self.x[3, 0])
        # msg.y = float(self.x[4, 0])
        # msg.heading = float(self.x[5, 0])
        # msg.propeller = self.propeller
        # msg.rudder = self.rudder
        # self.get_logger().info(f"x {float(self.x[3, 0])} y {float(self.x[4, 0])}")
            
        # self.state_pub.publish(msg)
        self.publish_odom()

    # Compute quaternion from euler angles
    def eul_to_quat(self, eul, order='ZYX', deg=False):
        quat = np.zeros(4, dtype=float)

        if order != 'ZYX':
            raise ValueError('Any order other than ZYX is not currently available!')

        # Write your code here

        if order == 'ZYX':
            
            if deg:
                phi = eul[0] * np.pi / 180
                theta = eul[1] * np.pi / 180
                psi = eul[2] * np.pi / 180
            else:
                phi = eul[0]
                theta = eul[1]
                psi = eul[2]

            quat[0] = np.cos(psi/2) * np.cos(theta/2) * np.cos(phi/2) + np.sin(psi/2) * np.sin(theta/2) * np.sin(phi/2)
            quat[1] = np.cos(psi/2) * np.cos(theta/2) * np.sin(phi/2) - np.sin(psi/2) * np.sin(theta/2) * np.cos(phi/2)
            quat[2] = np.sin(psi/2) * np.cos(theta/2) * np.sin(phi/2) + np.cos(psi/2) * np.sin(theta/2) * np.cos(phi/2)
            quat[3] = np.sin(psi/2) * np.cos(theta/2) * np.cos(phi/2) - np.cos(psi/2) * np.sin(theta/2) * np.sin(phi/2)

            quat = quat / np.linalg.norm(quat)

        return quat

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
