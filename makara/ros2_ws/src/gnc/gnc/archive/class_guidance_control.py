import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
from interfaces.msg import Actuator

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))

import module_kinematics as kin
import module_control as con
from class_geofence import GeoFence
from class_static_obstacle import StaticObstacle

import warnings
import scipy
from scipy.integrate import solve_ivp
import multiprocessing

class Guidance_Control():
    terminate_flag = False

    # Non-dimensionalizing parameters
    length = None
    U_des = None

    # State estimation parameters
    x_hat = None
    P_hat = None
    u_cmd = None

    y_p_int = 0.0
    u_err_int = 0.0

    odometry = {}
    actuator = {}

    current_time = 0.0

    geofences = []
    static_obstacles = []
    
    geo_grad = np.zeros(2)
    geo_active = False


    def __init__(self, topic_prefix, 
                rate=10, 
                gps_datum=np.array([12.99300425860631, 80.23913114094384, 94.0]),
                geofence=None,
                static_obstacles=None,                
                vessel=None,                
                odom_topics=None,
                gravity=9.80665,
                density=1000):

        self.g = gravity
        self.rho = density
        self.topic_prefix = topic_prefix
        self.rate = int(rate)
        self.llh0 = gps_datum
        
        self.vessel = vessel
        self.length = vessel.length
        self.Fn = vessel.Fn
        self.U_des = self.Fn * np.sqrt(self.g * self.length)

        self.dt = (1 / self.rate) * self.U_des / self.length

        self.euler_angle_flag = self.vessel.euler_angle_flag        
        
        if self.euler_angle_flag:
            self.x_hat = np.zeros(14)
            self.P_hat = np.eye(14)
            self.u_cmd = np.zeros(2)
        else:
            self.x_hat = np.zeros(15)
            self.x_hat[9] = 1.0
            self.P_hat = np.eye(15)
            self.u_cmd = np.zeros(2)
        
        waypoints_type = vessel.waypoints_type
        waypoints = vessel.waypoints

        # Convert GPS waypoints to NED waypoints
        if waypoints_type == 'GPS':
            waypoints_xyz = np.zeros_like(waypoints)            
            
            for i in range(waypoints.shape[0]):
                waypoints_xyz[i, :] = kin.llh_to_ned(waypoints[i, :], self.llh0)
            
            self.waypoints = waypoints_xyz
        else:
            self.waypoints = waypoints
        
        if static_obstacles is not None:
            for i in range(len(static_obstacles)):
                obstacle_llh = np.zeros(3)

                obstacle_llh[0] = static_obstacles[i][0]
                obstacle_llh[1] = static_obstacles[i][1]
                obstacle_llh[2] = static_obstacles[i][2]

                obstacle_xyz = kin.llh_to_ned(obstacle_llh, self.llh0)
                x0 = obstacle_xyz[0]
                y0 = obstacle_xyz[1]
                z0 = obstacle_xyz[2]

                self.static_obstacles.append(StaticObstacle(x0, y0, z0, id=i))

        if geofence is not None:            
            for i in range(len(geofence)-1):

                geofence_0 = np.zeros(3)
                geofence_1 = np.zeros(3)

                geofence_0[0] = geofence[i][0]; geofence_0[1] = geofence[i][1]
                geofence_1[0] = geofence[i + 1][0]; geofence_1[1] = geofence[i + 1][1]

                geofence_0[2] = self.llh0[2]; geofence_1[2] = self.llh0[2]

                geofence_0_xyz = kin.llh_to_ned(geofence_0, self.llh0)
                geofence_1_xyz = kin.llh_to_ned(geofence_1, self.llh0)
                
                x0 = geofence_0_xyz[0]; y0 = geofence_0_xyz[1]
                x1 = geofence_1_xyz[0]; y1 = geofence_1_xyz[1]

                self.geofences.append(GeoFence(x0, y0, x1, y1, id=i))

        self.guidance = self.vessel.guidance
        self.control = self.vessel.control

        if self.guidance is not None:
            self.current_waypoint = self.waypoints[0]
            self.goal_waypoint = self.waypoints[1]
            self.waypoint_index = 0

        if odom_topics is not None:
            self.odom_topic = odom_topics[0]
            self.enc_topic = odom_topics[1]
        else:
            self.odom_topic = f'/{self.topic_prefix}/odometry'
            self.enc_topic = f'/{self.topic_prefix}/encoders_odometry'
        
        self.node = Node(f'Guidance_{self.topic_prefix}')

        self.odometry['sub_odo'] = self.node.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)
        self.odometry['sub_enc'] = self.node.create_subscription(Actuator, self.enc_topic, self.encoder_callback, 10)

        self.actuator['pub'] = self.node.create_publisher(Actuator, f'/{self.topic_prefix}/actuator_cmd', self.rate)
        self.actuator['pub_geo'] = self.node.create_publisher(Bool, f'/{self.topic_prefix}/geofence_flag', self.rate)
        self.actuator['pub_static_obs'] = self.node.create_publisher(Bool, f'/{self.topic_prefix}/static_obstacle_flag', self.rate)
        self.actuator['timer'] = self.node.create_timer(1/self.rate, callback=self.publish_actuator)        

    def guidance_call(self):

        # Waypoint switching
        if np.linalg.norm(self.x_hat[6:9] - self.goal_waypoint / self.length) < 3 and self.terminate_flag is False:
            
            self.waypoint_index += 1

            if self.waypoint_index == self.waypoints.shape[0] - 1:
                self.terminate_flag = True

            else:                
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.goal_waypoint = self.waypoints[self.waypoint_index + 1]

        # Guidance mechanism
        self.pi_p = np.arctan2(self.goal_waypoint[1] - self.current_waypoint[1], 
                        self.goal_waypoint[0] - self.current_waypoint[0])

        R_n_p = np.array([[np.cos(self.pi_p), -np.sin(self.pi_p)], [np.sin(self.pi_p), np.cos(self.pi_p)]])
        
        x_n_i = self.current_waypoint[0:2] / self.length
        x_g_i = self.goal_waypoint[0:2] / self.length
        x_n = self.x_hat[6:8]

        # Along track distance of goal
        xy_g_e = R_n_p.T @ (x_g_i - x_n_i)
        self.x_g_e = xy_g_e[0]

        # Along and cross track distance of ship
        xy_p_e = R_n_p.T @ (x_n - x_n_i)
        self.x_p_e = xy_p_e[0]
        self.y_p_e = xy_p_e[1]

        # Calculating the u_err_int (for PI controller for propeller)
        U = np.linalg.norm(self.x_hat[0:2])
        ud_err_int = (1 - U)
        self.u_err_int = self.u_err_int + self.dt * ud_err_int

        if self.guidance == 'ilos':
            self.ilos_guidance()

    def control_call(self, override=False):

        if override:
            
            u_cmd = con.pid(self, rudder_flag=True, prop_flag=True, rudder_default=0.0, prop_default=0.0)
            self.rudder_cmd = u_cmd[0]
            self.propeller_cmd = u_cmd[1]

        else:
            
            if 'pid' in self.control:
                if self.control == 'pid':
                    u_cmd = con.pid(self, rudder_flag=True, prop_flag=True, rudder_default=0.0, prop_default=0.0)
                else:
                    if len(self.control.split('_')) > 2:
                        prop_default = np.float64(self.control.split('_')[1])
                        self.psi_des = np.float64(self.control.split('_')[2]) * np.pi / 180
                        u_cmd = con.pid(self, rudder_flag=True, prop_flag=False, rudder_default=0.0, prop_default=prop_default)
                    else:
                        prop_default = np.float64(self.control.split('_')[1])
                        u_cmd = con.pid(self, rudder_flag=True, prop_flag=False, rudder_default=0.0, prop_default=prop_default)
            
            if 'straight' in self.control:            
                prop_default = np.float64(self.control.split('_')[1])
                u_cmd = con.fixed(self, rudder_default=0.0, prop_default=prop_default)
            
            if 'turning_circle' in self.control:
                rudder_default = np.float64(self.control.split('_')[2])
                prop_default = np.float64(self.control.split('_')[3])
                exec_lead = np.float64(self.control.split('_')[4])
                u_cmd = con.turning_circle(self, rudder_default=rudder_default, prop_default=prop_default, exec_lead_time=exec_lead)
            
            if 'zigzag' in self.control:            
                psi_switch = np.float64(self.control.split('_')[1])
                delta_switch = np.float64(self.control.split('_')[2])
                prop_default = np.float64(self.control.split('_')[3])
                exec_lead_time = np.float64(self.control.split('_')[4])
                u_cmd = con.zigzag(self, psi_switch, delta_switch, prop_default, exec_lead_time)
                
            if 'spiral' in self.control:            
                delta_start = np.float64(self.control.split('_')[1])
                delta_step = np.float64(self.control.split('_')[2])
                delta_end = np.float64(self.control.split('_')[3])
                switch_time = np.float64(self.control.split('_')[4])
                prop_default = np.float64(self.control.split('_')[5])
                exec_lead_time = np.float64(self.control.split('_')[6])
                u_cmd = con.spiral(self, delta_start, delta_step, delta_end, switch_time, prop_default, exec_lead_time)

            if 'modified_spiral' in self.control:
                delta_start = np.float64(self.control.split('_')[2])
                delta_step = np.float64(self.control.split('_')[3])
                delta_end = np.float64(self.control.split('_')[4])
                switch_time = np.float64(self.control.split('_')[5])
                prop_default = np.float64(self.control.split('_')[6])
                exec_lead_time = np.float64(self.control.split('_')[7])
                u_cmd = con.modified_spiral(self, delta_start, delta_step, delta_end, switch_time, prop_default, exec_lead_time)

            self.rudder_cmd = u_cmd[0]
            self.propeller_cmd = u_cmd[1]


    def publish_actuator(self):

        self.current_time += self.dt

        self.static_obs_grad = np.zeros(3)
        self.static_obs_active = False

        if self.static_obstacles:
            curr_loc = self.x_hat[6:9] * self.length
            agent_x = curr_loc[0]
            agent_y = curr_loc[1]
            agent_z = curr_loc[2]

            for static_obstacle in self.static_obstacles:
                static_obstacle_active, static_obstacle_grad = static_obstacle.calc_static_obstacle_grad(agent_x, agent_y, agent_z)

                self.static_obs_active = self.static_obs_active or static_obstacle_active
                self.static_obs_grad = self.static_obs_grad + static_obstacle_grad
        
        self.geo_grad = np.zeros(2)
        self.geo_active = False

        if self.geofences:
            curr_loc = self.x_hat[6:8] * self.length
            agent_x = curr_loc[0]
            agent_y = curr_loc[1]
            
            for geofence in self.geofences:
                geofence_active, geofence_grad = geofence.calc_geofence_grad(agent_x, agent_y)

                self.geo_active = self.geo_active or geofence_active
                self.geo_grad = self.geo_grad + geofence_grad
        

        #==========================================================
        # Note that Geofence will override static obstacle guidance
        #==========================================================
        if self.geo_active:
            
            self.psi_des = np.arctan2(self.geo_grad[1], self.geo_grad[0])
            self.control_call(override=True)

        else:

            if self.static_obs_active:

                self.psi_des = np.arctan2(self.static_obs_grad[1], self.static_obs_grad[0])
                self.control_call(override=True)

            else:

                if self.guidance is not None:
                    self.guidance_call()

            self.control_call(override=False)

        current_time = self.node.get_clock().now()

        # Create Boolean message of active geofence
        msg = Bool()
        msg.data = self.geo_active
        self.actuator['pub_geo'].publish(msg)

        # Create Boolean message of active static obstacle
        msg = Bool()
        msg.data = self.static_obs_active and (not self.geo_active)
        self.actuator['pub_static_obs'].publish(msg)

        # Create Actuator message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        
        act.rudder = self.rudder_cmd * 180.0 / np.pi
        act.propeller = self.propeller_cmd * 60 * self.U_des / self.length

        self.actuator['pub'].publish(act)

        self.actuator_cmd = f'{act.rudder:.2f}, {act.propeller:.2f}'

        if self.euler_angle_flag:
            eul = self.x_hat[9:12] * 180 / np.pi
            quat = kin.eul_to_quat(eul, deg=True)
        else:
            quat = self.x_hat[9:13]
            eul = kin.quat_to_eul(quat, deg=True)
        
        if self.euler_angle_flag:
            rud_indx = 12
            prop_indx = 13
        else:
            rud_indx = 13
            prop_indx = 14
        
        print(f"Linear Velocity (m/s)    : {self.x_hat[0] * self.U_des:.4f}, {self.x_hat[1] * self.U_des:.4f}, {self.x_hat[2] * self.U_des:.4f}")
        print(f"Angular Velocity (rad/s) : {self.x_hat[3] * self.U_des / self.length:.4f}, {self.x_hat[4] * self.U_des / self.length:.4f}, {self.x_hat[5] * self.U_des / self.length:.4f}")
        print(f"Linear Position (m)      : {self.x_hat[6] * self.length:.4f}, {self.x_hat[7] * self.length:.4f}, {self.x_hat[8] * self.length:.4f}")
        print(f"Orientation (deg)        : {eul[0]:.2f}, {eul[1]:.2f}, {eul[2]:.2f}")
        print(f"Unit quaternion          : {quat[0]:.2f}, {quat[1]:.2f}, {quat[2]:.2f}, {quat[3]:.2f}")
        print(f"Rudder angle (deg)       : {self.x_hat[rud_indx] * 180 / np.pi:.2f}")
        print(f"Propeller Speed (RPM)    : {self.x_hat[prop_indx] * 60 * self.U_des / self.length:.2f}")

        if self.guidance is not None:

            print(f"Goal Waypoint (m)        : {self.goal_waypoint[0]:.2f}, {self.goal_waypoint[1]:.2f}, {self.goal_waypoint[2]:.2f} ")
            print(f"Distance to Goal (m)     : {np.linalg.norm(self.goal_waypoint - self.x_hat[6:9] * self.length):.2f}")
            print(f"Desired heading (deg)    : {self.psi_des * 180.0 / np.pi:.2f}")
        
        print('\n\n')

    def odometry_callback(self, msg):
        # Update x_hat based on odometry message
        self.x_hat[0] = msg.twist.twist.linear.x / self.U_des
        self.x_hat[1] = msg.twist.twist.linear.y / self.U_des
        self.x_hat[2] = msg.twist.twist.linear.z / self.U_des

        self.x_hat[3] = msg.twist.twist.angular.x / (self.U_des / self.length)
        self.x_hat[4] = msg.twist.twist.angular.y / (self.U_des / self.length)
        self.x_hat[5] = msg.twist.twist.angular.z / (self.U_des / self.length)

        self.x_hat[6] = msg.pose.pose.position.x / self.length
        self.x_hat[7] = msg.pose.pose.position.y / self.length
        self.x_hat[8] = msg.pose.pose.position.z / self.length
        
        # Extract quaternion orientation and convert to euler or set directly
        quat = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])

        if self.euler_angle_flag:
            euler_angles = kin.quat_to_eul(quat)  # Convert to euler if necessary
            self.x_hat[9:12] = euler_angles  # Store orientation
        else:
            self.x_hat[9:13] = euler_angles  # Store orientation
    
    def encoder_callback(self, msg):
        # Update x_hat based on encoder message
        if self.euler_angle_flag:
            self.x_hat[12] = msg.rudder * np.pi / 180.0
            self.x_hat[13] = msg.propeller / (60 * self.U_des / self.length)
        else:
            self.x_hat[13] = msg.rudder * np.pi / 180.0
            self.x_hat[14] = msg.propeller / (60 * self.U_des / self.length)

    def ilos_guidance(self):

        # Look ahead distance
        Delta = 2
        kappa = 0.05

        # Update the value of self.y_p_int (for ILOS guidance)
        yd_p_int = Delta * self.y_p_e / (Delta **2 + (self.y_p_e + kappa * self.y_p_int) ** 2)
        self.y_p_int = self.y_p_int + self.dt * yd_p_int

        # Compute the desired heading angle using self.y_p_e and self.y_p_int
        if self.x_p_e < self.x_g_e:
            self.psi_des = self.pi_p - np.arctan(self.y_p_e / Delta + kappa * self.y_p_int / Delta)
        else:
            self.psi_des = self.pi_p - np.pi + np.arctan(self.y_p_e / Delta + kappa * self.y_p_int / Delta)