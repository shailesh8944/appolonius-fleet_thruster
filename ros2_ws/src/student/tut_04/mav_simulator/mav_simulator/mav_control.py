import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import sys
sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03 import module_control as con
from ros2_ws.src.student.tut_03.module_kinematics import quat_to_eul
from ros2_ws.src.student.tut_03.read_input import read_input
import numpy as np

class MAVController(Node):
    def __init__(self, control_mode):
        super().__init__('mav_controller')
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'mav_odom',
            self.odom_callback,
            10)
            
        # Create publisher for rudder command
        self.rudder_pub = self.create_publisher(
            Float64,
            'mav_rudder_command',
            10)
            
        # Initialize time
        self.start_time = self.get_clock().now()
        
        # Select control mode
        self.control_mode = control_mode
        
        self.get_logger().info('MAV Controller initialized')

    def odom_callback(self, msg):
        # Get current time in seconds
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9

        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        eul = quat_to_eul(quat, order='ZYX')
        
        # Create state vector from odometry
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
            0.0 # rudder angle
        ])
        
        # Calculate control command
        rudder_cmd = self.control_mode(t, state)
        
        # Publish command
        cmd_msg = Float64()
        cmd_msg.data = float(rudder_cmd)
        self.rudder_pub.publish(cmd_msg)
        
        # Log info
        self.get_logger().info(
            f'\nTime: {t:.2f}s\n'            
            f'Rudder Command: {rudder_cmd*180/np.pi:.2f} deg\n'
        )

def main(args=None):
    rclpy.init(args=args)
    
    vessel_params, _ = read_input()
    control_type = vessel_params['control_type']
    
    if control_type == 'switching_rudder':
        control_mode = con.switching_rudder
    elif control_type == 'fixed_rudder':
        control_mode = con.fixed_rudder
    elif control_type == 'zero_rudder':
        control_mode = lambda t, state: con.fixed_rudder(t, state, rudder_angle=0.0)
    else:
        raise ValueError(f"Invalid control type: {control_type}")
    
    controller = MAVController(control_mode)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()