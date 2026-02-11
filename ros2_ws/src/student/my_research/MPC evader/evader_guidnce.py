
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
        self.received_rudder = 0.0
        # Subscribers
        self.create_subscription(Odometry, '/er/odometry', self.pose_cb, 10)
        self.create_subscription(Pose2D, '/pursuer1/pose', self.pursuer1_pose_callback, 10)
        self.create_subscription(Pose2D, '/pursuer2/pose', self.pursuer2_pose_callback, 10)
        self.create_subscription(Pose2D, '/pursuer3/pose', self.pursuer3_pose_callback, 10)
        self.create_subscription(Float32, '/kf/yaw', self.kf_yaw_callback, 10)
        self.create_subscription(Float32, '/evader/rudder_cmd', self.rudder_callback, 10)  # NEW
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
    def rudder_callback(self, msg: Float32):
        self.received_rudder = float(msg.data)
        self.get_logger().debug(f"Received rudder command: {self.received_rudder:.2f}°")
    def pursuer1_pose_callback(self, msg): self.pursuer1_pose = msg
    def pursuer2_pose_callback(self, msg): self.pursuer2_pose = msg
    def pursuer3_pose_callback(self, msg): self.pursuer3_pose = msg
    def clip(self, value, threshold):
        return max(min(value, threshold), -threshold)
    def control_loop(self):
        # Wait for valid pose
        if self.x == 0.0 and self.y == 0.0:
            self.get_logger().warn("Waiting for initial odometry...")
            return

       

        rudder_deg_rad = self.received_rudder
        rudder_deg = math.degrees(rudder_deg_rad)
        self.get_logger().debug(f"Received rudder command: {rudder_deg:.2f}°")
        rudder_deg = self.clip(rudder_deg, self.MAX_RUDDER_DEG)
        self.publish_actuator_command(self.RPM, rudder_deg)

   

    

    def publish_actuator_command(self, rpm, rudder_deg):
        m = Actuator()
        m.header.stamp = self.get_clock().now().to_msg()
        m.actuator_names = ['rudder', 'propeller']
        m.actuator_values = [float(rudder_deg), float(rpm)]
        self.actuator_publisher.publish(m)

    

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
