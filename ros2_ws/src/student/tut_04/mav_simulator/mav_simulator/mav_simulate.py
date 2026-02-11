import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

import sys
sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.class_vessel import Vessel
from ros2_ws.src.student.tut_03.module_kinematics import eul_to_quat
from ros2_ws.src.student.tut_03.read_input import read_input
import numpy as np

class MavNode(Node):
    def __init__(self, vessel_params, hydrodynamic_data):
        # Initialize the node
        super().__init__('mav_node')

        # Initialize the vessel
        self.vessel = Vessel(vessel_params, hydrodynamic_data, ros_flag=True)
        
        # Create a publisher to publish the vessel odometry
        self.publisher = self.create_publisher(Odometry, 'mav_odom', 10)

        # Create a publisher to publish the rudder angle
        self.rudder_publisher = self.create_publisher(Float64, 'mav_rudder', 10)

        # Create a subscriber to receive the rudder angle
        self.rudder_subscriber = self.create_subscription(Float64, 'mav_rudder_command', self.rudder_callback, 10)

        # Create a timer to call the timer_callback function at the vessel time step
        self.timer = self.create_timer(self.vessel.dt, self.timer_callback)        

    def timer_callback(self):
        
        # Step the vessel forward in time
        self.vessel.step()
        
        # Create an odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.vessel.current_state[6]
        odom_msg.pose.pose.position.y = self.vessel.current_state[7]
        odom_msg.pose.pose.position.z = self.vessel.current_state[8]
        
        # Set orientation
        q = eul_to_quat(self.vessel.current_state[9:12], order='ZYX', deg=False)
        odom_msg.pose.pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])
        
        # Set linear velocity
        odom_msg.twist.twist.linear.x = self.vessel.current_state[0]
        odom_msg.twist.twist.linear.y = self.vessel.current_state[1]
        odom_msg.twist.twist.linear.z = self.vessel.current_state[2]
        
        # Set angular velocity
        odom_msg.twist.twist.angular.x = self.vessel.current_state[3]
        odom_msg.twist.twist.angular.y = self.vessel.current_state[4]
        odom_msg.twist.twist.angular.z = self.vessel.current_state[5]
        
        # Publish the odometry message
        self.publisher.publish(odom_msg)

        # Publish rudder angle
        rudder_msg = Float64()
        rudder_msg.data = self.vessel.current_state[12] * 180 / np.pi
        self.rudder_publisher.publish(rudder_msg)

        # Log info
        self.get_logger().info(
            f'\nTime: {self.vessel.t:.2f}s\n'
            f'Position (x,y,z): [{self.vessel.current_state[6]:.2f}, {self.vessel.current_state[7]:.2f}, {self.vessel.current_state[8]:.2f}]\n'
            f'Euler angles (phi,theta,psi): [{self.vessel.current_state[9]*180/np.pi:.2f}, {self.vessel.current_state[10]*180/np.pi:.2f}, {self.vessel.current_state[11]*180/np.pi:.2f}]\n'
            f'Velocity (u,v,w): [{self.vessel.current_state[0]:.2f}, {self.vessel.current_state[1]:.2f}, {self.vessel.current_state[2]:.2f}]\n'
            f'Angular velocity (p,q,r): [{self.vessel.current_state[3]:.2f}, {self.vessel.current_state[4]:.2f}, {self.vessel.current_state[5]:.2f}]\n'
            f'Rudder Command: {self.vessel.current_state[12] * 180 / np.pi:.2f} deg\n'
        )

    def rudder_callback(self, msg):
        # Set the commanded rudder angle
        self.vessel.delta_c = msg.data

def main(args=None):
    try:
        # Initialize the ROS 2 node
        rclpy.init(args=args)
        vessel_params, hydrodynamic_data = read_input()
        vessel_node = MavNode(vessel_params, hydrodynamic_data)
        rclpy.spin(vessel_node)

    except KeyboardInterrupt:
        # Destroy the node and shutdown ROS 2
        vessel_node.destroy_node()
        rclpy.shutdown()

    finally:
        # Destroy the node and shutdown ROS 2
        vessel_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()