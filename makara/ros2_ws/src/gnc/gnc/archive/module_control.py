import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
from interfaces.msg import Actuator

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin

def fixed(gc, rudder_default=0.0, prop_default=0.0):
    
    rudder_cmd = rudder_default
    rudder_cmd = kin.clip(rudder_cmd, 35)

    propeller_cmd = prop_default
    propeller_cmd = kin.clip(propeller_cmd, 1000.0)

    u_cmd = np.array([rudder_cmd  * np.pi / 180, propeller_cmd * gc.length / (gc.U_des * 60.0)])

    return u_cmd

def turning_circle(gc, rudder_default=35.0, prop_default=1000.0, exec_lead_time=30.0):
    
    rudder_cmd = rudder_default
    rudder_cmd = kin.clip(rudder_cmd, 35)

    propeller_cmd = prop_default
    propeller_cmd = kin.clip(propeller_cmd, 1000.0)

    if gc.current_time < exec_lead_time * gc.U_des / gc.length:
        u_cmd = fixed(gc, rudder_default=0.0, prop_default=propeller_cmd)
    else:
        u_cmd = fixed(gc, rudder_default=rudder_cmd, prop_default=propeller_cmd)

    return u_cmd

def spiral(gc, delta_start=0.0, delta_step=5.0, delta_end=35.0, switch_time=300, prop_default=1000.0, exec_lead_time=30.0):
    
    t = gc.current_time

    delta_list = np.arange(delta_start, delta_end + delta_step, delta_step)
    m = np.floor(t / switch_time).astype(int)
    
    if m < len(delta_list):
        delta = delta_list[m]
    else:
        delta = delta_list[-1]
    
    rudder_cmd = delta
    rudder_cmd = kin.clip(rudder_cmd, 35)

    propeller_cmd = prop_default
    propeller_cmd = kin.clip(propeller_cmd, 1000.0)

    if gc.current_time < exec_lead_time * gc.U_des / gc.length:
        u_cmd = fixed(gc, rudder_default=0.0, prop_default=propeller_cmd)
    else:
        u_cmd = fixed(gc, rudder_default=rudder_cmd, prop_default=propeller_cmd)

    return u_cmd

def modified_spiral(gc, delta_start=0.0, delta_step=5.0, delta_end=35.0, switch_time=300, prop_default=1000.0, exec_lead_time=30.0):
    
    t = gc.current_time
    
    delta_list = np.arange(delta_start, delta_end + delta_step, delta_step)
    delta_list[1::2] = -delta_list[1::2]
    
    delta_list_new = np.zeros(4 * len(delta_list) - 1)
    delta_list_new[0::4] = delta_list
    delta_list_new[2::4] = delta_list
    delta_list_new[1::4] = -delta_list / 2
    
    m = int(np.floor(t / switch_time))
    
    if m < len(delta_list_new):
        delta = delta_list_new[m]
    else:
        delta = delta_list_new[-1]
    
    rudder_cmd = delta
    rudder_cmd = kin.clip(rudder_cmd, 35)

    propeller_cmd = prop_default
    propeller_cmd = kin.clip(propeller_cmd, 1000.0)

    if gc.current_time < exec_lead_time * gc.U_des / gc.length:
        u_cmd = fixed(gc, rudder_default=0.0, prop_default=propeller_cmd)
    else:
        u_cmd = fixed(gc, rudder_default=rudder_cmd, prop_default=propeller_cmd)

    return u_cmd

def zigzag(gc, psi_switch=20.0, delta_switch=10.0, prop_default=1000.0, exec_lead_time=30.0):
    
    rudder_cmd = delta_switch
    rudder_cmd = kin.clip(rudder_cmd, 35)

    propeller_cmd = prop_default
    propeller_cmd = kin.clip(propeller_cmd, 1000.0)

    if gc.euler_angle_flag:
        eul = gc.x_hat[9:12] * 180 / np.pi        
    else:
        quat = gc.x_hat[9:13]
        eul = kin.quat_to_eul(quat, deg=True)
    
    psi = eul[2]
    r = gc.x_hat[5] * gc.U_des / gc.length

    if gc.current_time < exec_lead_time * gc.U_des / gc.length:
        u_cmd = fixed(gc, rudder_default=0.0, prop_default=propeller_cmd)
    else:
        
        if r > 0 and psi < psi_switch and psi > -psi_switch:
            rudder_cmd = delta_switch
        elif psi >= psi_switch:
            rudder_cmd = -delta_switch
        elif r <= 0 and psi <= psi_switch and psi >= -psi_switch:
            rudder_cmd = -delta_switch
        elif psi < -psi_switch:
            rudder_cmd = delta_switch
        else:
            rudder_cmd = 0  # Default case if none of the conditions match

        u_cmd = fixed(gc, rudder_default=rudder_cmd, prop_default=propeller_cmd)

    return u_cmd

def pid(gc, rudder_flag=True, prop_flag=True, rudder_default=0.0, prop_default=0.0):

    U = np.linalg.norm(gc.x_hat[0:2])
    
    r = gc.x_hat[5]

    if gc.euler_angle_flag:
        eul = gc.x_hat[9:12]
    else:
        eul = kin.quat_to_eul(gc.x_hat[9:13])
    
    psi = eul[2]

    if not gc.terminate_flag:
        
        Kp_R = 4
        Kd_R = 2

        rudder_cmd = 0.0
        rudder_cmd = Kp_R * kin.ssa(gc.psi_des - psi) - Kd_R * r
        rudder_cmd = kin.clip(rudder_cmd, 35 * np.pi / 180)

        Kp_P = 100
        Ki_P = 10

        propeller_cmd = Kp_P * (1 - U) + Ki_P * gc.u_err_int
        propeller_cmd = kin.clip(propeller_cmd, 1000.0 * gc.length / (gc.U_des * 60.0))

    else:

        rudder_cmd = 0.0 * np.pi / 180.0
        propeller_cmd = 0.0 * gc.length / (gc.U_des * 60.0)
    
    u_cmd = np.array([rudder_cmd, propeller_cmd])

    if not rudder_flag and not gc.terminate_flag:
        u_cmd[0] = kin.clip(rudder_default, 35) * np.pi / 180
    
    if not prop_flag and not gc.terminate_flag:
        u_cmd[1] = kin.clip(prop_default, 1000.0) * gc.length / (gc.U_des * 60.0)

    return u_cmd