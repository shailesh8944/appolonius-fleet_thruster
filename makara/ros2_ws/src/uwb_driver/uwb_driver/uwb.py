#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cmath
import numpy as np
from serial import Serial
import time
# import matplotlib.pyplot as plt
import json
import re
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Point, Quaternion

from mav_simulator.class_world import World


# USB PORT

L = 31.6
B = 20.6
c1 = 2.5
# L = 2.24
# B = 1.12
# c1 = 0.66

def P123(r1,r2,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r2**2 - r3**2)/(2*L), (B**2 + r1**2 - r2**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)/(2*B*L))
        return [s[0],s[1]]
    
def P124(r1,r2,r4):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r1**2 - r4**2)/(2*L), (B**2 + r1**2 - r2**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r1**4 + 2*L**2*r1**2*r2**2 - L**2*r2**4)/(2*B*L))
        return [s[0],s[1]]
    
def P143(r1,r4,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)
    if f.real == 0:
        return None
    if f.real != 0:
        s = ((L**2 + r1**2 - r4**2)/(2*L), -(-B**2 + r3**2 - r4**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r1**2 + 2*B**2*L**2*r3**2 - B**2*r1**4 + 2*B**2*r1**2*r4**2 - B**2*r4**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)/(2*B*L))
        return [s[0],s[1]]
    
def P243(r2,r4,r3):
    f = cmath.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)
    if f.real == 0:
        return None
    if f.real != 0: 
        s = ((L**2 + r2**2 - r3**2)/(2*L), -(-B**2 + r3**2 - r4**2)/(2*B), c1 - np.sqrt(-B**4*L**2 - B**2*L**4 + 2*B**2*L**2*r2**2 + 2*B**2*L**2*r4**2 - B**2*r2**4 + 2*B**2*r2**2*r3**2 - B**2*r3**4 - L**2*r3**4 + 2*L**2*r3**2*r4**2 - L**2*r4**4)/(2*B*L))
        return [s[0],s[1]]


class UWB(Node):
    def __init__(self):
        super().__init__('uwb')

        # Get the namespace from the node
        self.declare_parameter('topic_prefix', '')
        self.topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value

        # Declare and retrieve the 'uwb_url' parameter
        self.declare_parameter('uwb_url', '/dev/uwb')  # Default value
        uwb_url = self.get_parameter('uwb_url').get_parameter_value().string_value

        # Log the received value
        self.get_logger().info(f'UWB URL received: {uwb_url}')

        portName='/dev/uwb'
        #portName= uwb_url
        # Create Serial object:
        self.ser = Serial(portName) # Also opens the port during object creation
        self.ser.close() # To set parameters

        # Change settings for Arduino default Serial.begin:
        self.ser.baudrate=115200
        self.ser.port=portName
        self.ser.bytesize=8
        self.ser.parity='N'
        self.ser.stopbits=1
        self.ser.timeout=0.1
        # Open port
        self.ser.open()
        self.ser.reset_input_buffer()


        self.pub = self.create_publisher(Vector3, f'{self.topic_prefix}/uwb/loc', 10)
        self.l = self.create_timer(timer_period_sec= 0.05, callback= self.uwb_loc_pub)
        self.odom = self.create_publisher(PoseWithCovarianceStamped, f'{self.topic_prefix}/uwb', 10)
        self.timer = self.create_timer(0.1, self.odom_callback)
        
        self.pose = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
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
    
    def odom_callback(self, ):
        if self.pose is None:
            return
        
        msg = PoseWithCovarianceStamped()
        self.z = 0.0
        msg.header.stamp =  self.get_clock().now().to_msg()
        msg.pose.pose.position = Point(x=self.x, y=self.y, z=self.z)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.pose.covariance = np.zeros(36)

        self.odom.publish(msg)
    
    def uwb_loc_pub(self, ):
        try:
            # self.ser.reset_input_buffer()
            N=self.ser.in_waiting        
            data=self.ser.readline() 
            data=data.decode('utf-8')
            f = json.loads(data)
            dis = {}
            for i in f['links']:
                if int(i["A"])==1786:
                    dis["r1"]=float(i["R"])
                if int(i["A"])==1787:
                    dis["r2"]=float(i["R"])
                if int(i["A"])==1788:
                    dis["r3"]=float(i["R"])
                if int(i["A"])==1789:
                    dis["r4"]=float(i["R"])
            # print(dis)
            if len(dis)<3:
                self.get_logger().info(f"No Fix.  {len(dis)} anchors detected")
                print(f"No Fix.  {len(dis)} anchors detected")
                
            elif len(dis)==3:
                print("3d fix")
                try:
                    if set(dis.keys()) == {'r1','r2','r3'}:
                        fs = P123(dis['r1'],dis['r2'],dis['r3'])
                    if set(dis.keys()) == {'r1','r2','r4'}:
                        fs = P124(dis['r1'],dis['r2'],dis['r4'])
                    if set(dis.keys()) == {'r1','r4','r3'}:
                        fs = P143(dis['r1'],dis['r4'],dis['r3'])
                    if set(dis.keys()) == {'r2','r4','r3'}:
                        fs = P243(dis['r2'],dis['r4'],dis['r3'])
                    print(fs)
                    uwbmsg = Vector3()

                    x_ned, y_ned, z = self.eul_to_rotm([
                    #0, 0, 0
                        np.pi, 0, -np.pi/2
                    ])@np.array([fs[0], fs[1], 0])

                    uwbmsg.x = x_ned+B
                    uwbmsg.y = y_ned+L
                    self.x = x_ned+B
                    self.y = y_ned+L
                    # uwbmsg.x = fs[0]
                    # uwbmsg.y = fs[1]
                    self.pose = fs
                    self.pub.publish(uwbmsg)
                    
                except RuntimeError:
                    print("value error")
            elif len(dis)==4:
                try:
                    print("4d Fix")
                    
                    fs1 = P123(dis['r1'],dis['r2'],dis['r3'])
                    fs2 = P124(dis['r1'],dis['r2'],dis['r4'])
                    fs3 = P143(dis['r1'],dis['r4'],dis['r3'])
                    fs4 = P243(dis['r2'],dis['r4'],dis['r3'])
                    fsx = np.mean([fs1[0],fs2[0],fs3[0],fs4[0]])
                    fsy = np.mean([fs1[1],fs2[1],fs3[1],fs4[1]])
                    fs = [fsx,fsy]
                    print(fs)
                
                    uwbmsg = Vector3()
                    # uwbmsg.x = fs[0]
                    # uwbmsg.y = fs[1]
                    x_ned, y_ned, z = self.eul_to_rotm([
                            #0, 0, 0
                            np.pi, 0, -np.pi/2
                        ])@np.array([fs[0], fs[1], 0])

                    uwbmsg.x = x_ned+B
                    uwbmsg.y = y_ned+L
                    self.x = x_ned+B
                    self.y = y_ned+L

                    self.pose = fs

                    self.pub.publish(uwbmsg)
                   
                except RuntimeError:
                    print("value error")
            # print(dis)
        except KeyboardInterrupt:
            return
        except Exception as e:
            pass
def main(args=None):

    rclpy.init(args=args)
    node = UWB()

    try:
        node.get_logger().info(f"UWB Started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    self.ser.close()

















