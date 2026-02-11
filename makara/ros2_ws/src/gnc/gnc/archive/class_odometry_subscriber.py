import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin

class OdometrySubscriber():
    trajectory = []
    encoders = []
    odom_topic = None
    encoders_topic = None
    node = None

    def __init__(self, odom_topic=None, encoders_topic=None):
        # super().__init__('webapp_subscriber')
        self.node = Node('webapp_subscriber')
        
        self.odom_topic = odom_topic
        self.encoders_topic = encoders_topic

        if self.odom_topic is not None:
            self.odom_subscription = self.node.create_subscription(
                Odometry,
                self.odom_topic,
                self.odometry_callback,
                10
            )
        
        if self.encoders_topic is not None:
            self.encoders_subscription = self.node.create_subscription(
                Actuator,
                self.encoders_topic,
                self.encoders_callback,
                10
            )
        
        now = self.node.get_clock().now().to_msg()
        self.t0 = now.sec + now.nanosec * 1.0e-9
    
    def odometry_callback(self, msg):
        
        t = np.float64(msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9) - self.t0

        position = msg.pose.pose.position
        
        orientation = msg.pose.pose.orientation
        quat = np.array([orientation.w, orientation.x, orientation.y, orientation.z])
        eul = kin.quat_to_eul(quat, deg=True)
        
        vel = msg.twist.twist.linear
        
        ang_vel = msg.twist.twist.angular
        # logging.info(f'{t}, {vel.x}, {vel.y}, {vel.z}, {ang_vel.x}, {ang_vel.y}, {ang_vel.z}, {position.x}, {position.y}, {position.z}, {eul[0]}, {eul[1]}, {eul[2]}')
        
        self.trajectory.append([
                t,
                vel.x, vel.y, vel.z,
                ang_vel.x, ang_vel.y, ang_vel.z,
                position.x, position.y, position.z,
                eul[0], eul[1], eul[2]
            ])

    def encoders_callback(self, msg):        

        t = np.float64(msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9) - self.t0

        rudder = msg.rudder
        propeller = msg.propeller
        
        self.encoders.append([t, rudder, propeller])