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
import module_kinematic_kf as kkf
import warnings
import scipy
from scipy.integrate import solve_ivp
import multiprocessing

class GNC():
    # This class creates a ROS2 node and initiates the callbacks for
    # IMU, GPS, UWB sensors and encoders. The class also initiates 
    # a publisher on /<vessel_name>_<vessel_id>/actuator_cmd based
    # on the guidance and control laws you will program. 

    sensor_sub = []
    odometry = {}
    actuator = {}
    rate = 10
    node = None
    topic_prefix = None
    actuator_cmd = '0.0, 0.0'

    # Non-dimensionalizing parameters
    length = None
    U_des = None

    # State estimation parameters
    x_hat = None
    P_hat = None
    u_cmd = None
    
    # Guidance parameters
    psi_des = 0.0

    # Control commands
    rudder_cmd = 0.0
    propeller_cmd = 0.0

    # Terminate flag
    terminate_flag = False

    y_p_int = 0.0
    u_err_int = 0.0
    
    current_time = 0.0
    score = 0.0

    g = 9.80665
    rho = 1000

    def __init__(self, topic_prefix, 
                rate=10, 
                sensors=[], 
                gps_datum=np.array([12.99300425860631, 80.23913114094384, 94.0]),
                length=3.147,
                Fn=0.20,
                waypoint_type='XYZ',
                waypoints=None,
                vessel_data=None,
                vessel_ode=None,
                euler_angle_flag=False,
                gnc_flag='gnc',
                kinematic_kf_flag=True,
                gravity=9.80665,
                density=1000):
        
        self.g = gravity
        self.rho = density
        self.topic_prefix = topic_prefix
        self.rate = int(rate)
        self.sensors = sensors
        self.llh0 = gps_datum
        self.gnc_flag = gnc_flag
        
        self.length = length
        self.Fn = Fn
        self.U_des = self.Fn * np.sqrt(9.80665 * self.length)

        # Convert GPS waypoints to NED waypoints
        if waypoint_type == 'GPS':
            waypoints_xyz = np.zeros_like(waypoints)
            
            for i in range(waypoints.shape()[0]):
                waypoints_xyz[i, :] = kin.llh_to_ned(waypoints[i, :], self.llh0)
            
            self.waypoints = waypoints_xyz        
        else:
            self.waypoints = waypoints

        self.current_waypoint = self.waypoints[0]
        self.goal_waypoint = self.waypoints[1]
        self.waypoint_index = 0

        self.vessel_data = vessel_data
        self.vessel_ode = vessel_ode
        self.euler_angle_flag = euler_angle_flag
        self.kinematic_kf_flag =  kinematic_kf_flag
        
        if self.kinematic_kf_flag:
            if self.euler_angle_flag:
                self.x_hat = np.zeros(17)
                self.P_hat = np.eye(17)
                self.u_cmd = np.zeros(2)
            else:
                self.x_hat = np.zeros(18)
                self.x_hat[9] = 1.0
                self.P_hat = np.eye(18)
                self.u_cmd = np.zeros(2)
        else:
            if self.euler_angle_flag:
                self.x_hat = np.zeros(14)
                self.P_hat = np.eye(14)
                self.u_cmd = np.zeros(2)
            else:
                self.x_hat = np.zeros(15)
                self.x_hat[9] = 1.0
                self.P_hat = np.eye(15)
                self.u_cmd = np.zeros(2)
        
        self.node = Node(f'GNC_{self.topic_prefix}')
        
        # Allocate ID to each sensor
        for i in range(len(self.sensors)):
            self.sensors[i]['id'] = i

        # Create an empty measurements array
        self.sensor_measurements = [None]*len(self.sensors)

        # Register the sensors
        if len(self.sensors) != 0:
            self.register_sensors()
        
        # Register the actuator
        if self.gnc_flag == "gnc":
            self.register_actuator()

        # Register odometry publisher
        self.register_odometry()
        
    def register_odometry(self):
        self.odometry['pub'] = self.node.create_publisher(Odometry, f'/{self.topic_prefix}/odometry', self.rate)
        self.odometry['timer'] = self.node.create_timer(1/self.rate, callback=self.publish_odometry)

    def register_actuator(self):
        self.actuator['pub'] = self.node.create_publisher(Actuator, f'/{self.topic_prefix}/actuator_cmd', self.rate)
        self.actuator['timer'] = self.node.create_timer(1/self.rate, callback=self.publish_actuator)

    def register_sensors(self):
        for i in range(len(self.sensors)):
            
            sensor_type = self.sensors[i]['sensor_type']
            
            if sensor_type == "IMU":

                topic_imu = self.sensors[i]['topic']
                arg_imu = i
                callback_imu = lambda x: self.imu_callback(x, arg_imu)
                h = self.node.create_subscription(Imu, topic_imu, callback_imu, 1)

            elif sensor_type == "GPS":
                
                topic_gps = self.sensors[i]['topic']
                arg_gps = i
                callback_gps = lambda x: self.gps_callback(x, arg_gps)
                h = self.node.create_subscription(NavSatFix, topic_gps, callback_gps, 1)

            elif sensor_type == "UWB":

                topic_uwb = self.sensors[i]['topic']
                arg_uwb = i
                callback_uwb = lambda x: self.uwb_callback(x, arg_uwb)
                h = self.node.create_subscription(PoseWithCovarianceStamped, topic_uwb, callback_uwb, 1)

            elif sensor_type == "encoders":

                topic_encoders = self.sensors[i]['topic']
                arg_encoders = i
                callback_encoders = lambda x: self.encoders_callback(x, arg_encoders)
                h = self.node.create_subscription(Actuator, topic_encoders, callback_encoders, 1)

            else:

                h = None
                warnings.warn(f"Sensor type {sensor_type} is unknown")

            self.sensor_sub.append(h)
    
    def jacobian(self, fun, x0):
        f0 = fun(x0)
        jacob = np.zeros((np.size(f0),np.size(x0)))
        eps = 1e-4

        for i in range(np.size(x0)):
            x2 = np.copy(x0)
            x2[i] = x2[i] + eps

            x1 = np.copy(x0)
            x1[i] = x1[i] - eps

            f1 = fun(x1)
            f2 = fun(x2)
            jacob[:, i] = (f2 - f1) / (2 * eps)
        
        return jacob

    def imu_callback(self, msg, sensor_id):

        self.sensor_measurements[sensor_id] = msg

        if self.euler_angle_flag:
            
            if self.kinematic_kf_flag:
                q = 9; p = 2; n = 17
            else:
                q = 6; p = 2; n = 14

            y = np.zeros(q)
            quat = np.zeros(4)

            y[0] = msg.angular_velocity.x / (self.U_des / self.length)
            y[1] = msg.angular_velocity.y / (self.U_des / self.length)
            y[2] = msg.angular_velocity.z / (self.U_des / self.length)
            
            quat[0] = msg.orientation.w
            quat[1] = msg.orientation.x
            quat[2] = msg.orientation.y
            quat[3] = msg.orientation.z

            eul = kin.quat_to_eul(quat)
            y[3:6] = eul

        else:

            if self.kinematic_kf_flag:
                q = 10; p = 2; n = 18
            else:
                q = 7; p = 2; n = 15

            y = np.zeros(q)

            y[0] = msg.angular_velocity.x / (self.U_des / self.length)
            y[1] = msg.angular_velocity.y / (self.U_des / self.length)
            y[2] = msg.angular_velocity.z / (self.U_des / self.length)
            y[3] = msg.orientation.w
            y[4] = msg.orientation.x
            y[5] = msg.orientation.y
            y[6] = msg.orientation.z
        
        if self.kinematic_kf_flag:
            a_ni_i = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            q_i_n = np.array([
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z
            ])

            a_ni_n = kin.quat_to_rotm(q_i_n) @ a_ni_i - np.array([0, 0, -self.g])
            a_ni_i = kin.quat_to_rotm(q_i_n).T @ a_ni_n
        
        # The following old code seems wrong as sensor_orientation is specified in terms of Euler angles Theta_bi, which leads to the q_i_b
        # This is consistent with Euler angles Theta_nb leading to quaternion q_b_n

        # q_b_i = self.sensors[sensor_id]['sensor_orientation']
        # R_b_i = kin.quat_to_rotm(q_b_i)
        # R_i_b = R_b_i.T

        r_bi_b = np.array(self.sensors[sensor_id]['sensor_location'])
        q_i_b = np.array(self.sensors[sensor_id]['sensor_orientation'])
        R_i_b = kin.quat_to_rotm(q_i_b)
        q_b_i = kin.quat_conjugate(q_i_b)
        R_b_i = R_i_b.T

        # Measurement Noise
        Rd = np.eye(q)
        Rd[0:3][:, 0:3] = msg.angular_velocity_covariance.reshape((3, 3)) / ( (self.U_des / self.length) ** 2)
        
        if np.abs(np.linalg.det(Rd[0:3][:, 0:3])) < 1e-3:
            
            # These values are based on maximum value from run_01 of sbg turntable test
            # https://github.com/MarineAutonomy/sbg_turntable/blob/main/ros2_ws/data/2024_09_08/run_01_output_csv/run_01.mat
            Rd[0:3][:, 0:3] = np.diag(np.array([0.01, 0.01, 0.01]) ** 2)
            
            # raise ValueError('Covariance of Angular Velocity Measurement is Singular')

        if self.euler_angle_flag:
            Rd[3:6][:, 3:6] = msg.orientation_covariance.reshape((3, 3))

        else:            
            Ceul = msg.orientation_covariance.reshape((3, 3))
            G = kin.dquat_deul(y[3:7])
            Cq = G @ Ceul @ G.T
            Rd[3:7][:, 3:7] = (Cq + Cq.T) / 2.0

        if self.kinematic_kf_flag:
            Rd[-3:][:, -3:] = msg.linear_acceleration_covariance.reshape((3, 3)) / ( self.U_des ** 2 / self.length )
            
            if np.abs(np.linalg.det(Rd[-3:][:, -3:])) < 1e-3:

                # These values are based on maximum value from run_01 of sbg turntable test
                # https://github.com/MarineAutonomy/sbg_turntable/blob/main/ros2_ws/data/2024_09_08/run_01_output_csv/run_01.mat                
                Rd[-3:][:, -3:] = np.diag(np.array([0.15, 0.15, 0.15]) ** 2)

                # raise ValueError('Covariance of Linear Acceleration Measurement is Singular')

        # Feed forward matrix
        Dd = np.zeros((q, p))

        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR IMU MEASUREMENTS
        ################################################################################
        
        if self.euler_angle_flag:
            # Output matrix
            Cd = np.zeros((q, n))

            x0 = kin.quat_to_eul(quat)  # Measured quaternion by IMU --> q_i_n --> Theta_ni
            
            # The transpose seems to be wrong in the previous version of the code
            # fun = lambda x: kin.rotm_to_eul(kin.eul_to_rotm(x) @ kin.quat_to_rotm(q_b_i).T)
            
            fun = lambda Theta_ni: kin.rotm_to_eul(kin.eul_to_rotm(Theta_ni) @ kin.quat_to_rotm(q_b_i))
            jacob = self.jacobian(fun, x0)
            
            Cd[0:3][:, 3:6] = R_i_b
            Cd[3:6][:, 9:12] = jacob

        else:
            # Output matrix
            Cd = np.zeros((q, n))

            qw = q_b_i[0]
            qx = q_b_i[1]
            qy = q_b_i[2]
            qz = q_b_i[3]
            
            Cd[0:3][:, 3:6] = R_i_b
            Cd[3:7][:, 9:13] = np.array([
                [qw, -qx, -qy, -qz],
                [qx,  qw,  qz, -qy],
                [qy, -qz,  qw,  qx],
                [qz,  qy, -qx,  qw]
            ])
            # The above is the matrix [A] corresponding to "quaternion product" {q1} * {q2} = [A]{q1}

        if self.kinematic_kf_flag:
            w_nb_b = self.x_hat[3:6]
            Cd[-3:][:, -3:] = R_b_i
            Cd[-3:][:, 3:6] = kkf.d_by_dw_nb_b_a_ni_i(w_nb_b, r_bi_b, q_i_b)
        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd, sensor='IMU')
    
    def gps_callback(self, msg, sensor_id):
        self.sensor_measurements[sensor_id] = msg

        q = 3; p = 2; 
        
        if self.euler_angle_flag:
            if self.kinematic_kf_flag:
                n = 17
            else:
                n = 14
        else:
            if self.kinematic_kf_flag:
                n = 18
            else:
                n = 15

        y = np.zeros(q)

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        llh = np.array([lat, lon, alt])

        ned = kin.llh_to_ned(llh, self.llh0)

        y[0] = ned[0]
        y[1] = ned[1]
        y[2] = ned[2]

        y = y / self.length

        # Measurement Noise
        Rd = np.array(msg.position_covariance).reshape((3, 3))  / (self.length ** 2)

        # Feed forward matrix
        Dd = np.zeros((q, p))

        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR GPS MEASUREMENTS
        ################################################################################
        
        # Output matrix
        Cd = np.zeros((q, n))
        Cd[0, 6] = 1
        Cd[1, 7] = 1
        Cd[2, 8] = 1

        sen_loc = self.sensors[sensor_id]['sensor_location']
        x_sen = sen_loc[0] / self.length
        y_sen = sen_loc[1] / self.length
        z_sen = sen_loc[2] / self.length
        r_sen = np.array([x_sen, y_sen, z_sen])

        if self.euler_angle_flag:
            eul = self.x_hat[9:12]
            fun = lambda x: kin.eul_to_rotm(x) @ r_sen
            jacob = self.jacobian(fun, eul)

            Cd[0:3][:, 9:12] = jacob
        
        else:
            qw = self.x_hat[9]
            qx = self.x_hat[10]
            qy = self.x_hat[11]
            qz = self.x_hat[12]

            Cd[0, 9]  = -2 * qz * y_sen + 2 * qy * z_sen
            Cd[0, 10] = 2 * qy * y_sen + 2 * qz * z_sen
            Cd[0, 11] = -4 * qy * x_sen + 2 * qx * y_sen + 2 * qw * z_sen
            Cd[0, 12] = -4 * qz * x_sen - 2 * qw * y_sen + 2 * qx * z_sen
            
            Cd[1, 9]  = 2 * qz * x_sen - 2 * qx * z_sen
            Cd[1, 10] = 2 * qy * x_sen - 4 * qx * y_sen - 2 * qw * z_sen
            Cd[1, 11] = 2 * qx * x_sen + 2 * qz * z_sen
            Cd[1, 12] = 2 * qw * x_sen - 4 * qz * y_sen + 2 * qy * z_sen

            Cd[2, 9]  = -2 * qy * x_sen + 2 * qx * y_sen
            Cd[2, 10] = 2 * qz * x_sen + 2 * qw * y_sen - 4 * qx * z_sen
            Cd[2, 11] = -2 * qw * x_sen + 2 * qz * y_sen - 4 * qy * z_sen
            Cd[2, 12] = 2 * qx * x_sen + 2 * qy * y_sen

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)
    
    def uwb_callback(self, msg, sensor_id):
        
        self.sensor_measurements[sensor_id] = msg

        q = 3; p = 2; 
        
        if self.euler_angle_flag:
            if self.kinematic_kf_flag:
                n = 17
            else:
                n = 14
        else:
            if self.kinematic_kf_flag:
                n = 18
            else:
                n = 15

        y = np.zeros(q)

        y[0] = msg.pose.pose.position.x
        y[1] = msg.pose.pose.position.y
        y[2] = msg.pose.pose.position.z

        y = y / self.length

        # Measurement Noise
        Rd_full = msg.pose.covariance.reshape((6, 6))
        Rd = Rd_full[0:3][:, 0:3] / (self.length ** 2)

        # Feed forward matrix
        Dd = np.zeros((q, p))
        
        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR UWB MEASUREMENTS
        ################################################################################

        # Output matrix
        Cd = np.zeros((q, n))

        sen_loc = self.sensors[sensor_id]['sensor_location']
        x_sen = sen_loc[0] / self.length
        y_sen = sen_loc[1] / self.length
        z_sen = sen_loc[2] / self.length
        r_sen = np.array([x_sen, y_sen, z_sen])

        Cd[0, 6] = 1
        Cd[1, 7] = 1
        Cd[2, 8] = 1

        if self.euler_angle_flag:
            eul = self.x_hat[9:12]
            fun = lambda x: kin.eul_to_rotm(x) @ r_sen
            jacob = self.jacobian(fun, eul)

            Cd[0:3][:, 9:12] = jacob
        
        else:
            qw = self.x_hat[9]
            qx = self.x_hat[10]
            qy = self.x_hat[11]
            qz = self.x_hat[12]            

            Cd[0, 9]  = -2 * qz * y_sen + 2 * qy * z_sen
            Cd[0, 10] = 2 * qy * y_sen + 2 * qz * z_sen
            Cd[0, 11] = -4 * qy * x_sen + 2 * qx * y_sen + 2 * qw * z_sen
            Cd[0, 12] = -4 * qz * x_sen - 2 * qw * y_sen + 2 * qx * z_sen
            
            Cd[1, 9]  = 2 * qz * x_sen - 2 * qx * z_sen
            Cd[1, 10] = 2 * qy * x_sen - 4 * qx * y_sen - 2 * qw * z_sen
            Cd[1, 11] = 2 * qx * x_sen + 2 * qz * z_sen
            Cd[1, 12] = 2 * qw * x_sen - 4 * qz * y_sen + 2 * qy * z_sen

            Cd[2, 9]  = -2 * qy * x_sen + 2 * qx * y_sen
            Cd[2, 10] = 2 * qz * x_sen + 2 * qw * y_sen - 4 * qx * z_sen
            Cd[2, 11] = -2 * qw * x_sen + 2 * qz * y_sen - 4 * qy * z_sen
            Cd[2, 12] = 2 * qx * x_sen + 2 * qy * y_sen

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)
        
    
    def encoders_callback(self, msg, sensor_id):
        
        self.sensor_measurements[sensor_id] = msg

        q = 2; p = 2; 
        
        if self.euler_angle_flag:
            if self.kinematic_kf_flag:
                n = 17
            else:
                n = 14
        else:
            if self.kinematic_kf_flag:
                n = 18
            else:
                n = 15

        y = np.zeros(q)

        y[0] = msg.rudder * np.pi / 180
        y[1] = msg.propeller / 60

        y[1] = y[1] * self.length / self.U_des

        # Measurement Noise
        Rd = msg.covariance.reshape((2, 2))
        Rd[0, 1] = Rd[0, 1] / (self.U_des / self.length)
        Rd[1, 0] = Rd[1, 0] / (self.U_des / self.length)
        Rd[1, 1] = Rd[1, 1] / ((self.U_des / self.length) ** 2)

        # Feed forward matrix
        Dd = np.zeros((q, p))
        
        ################################################################################
        # UPDATE THE OUTPUT MATRIX FOR ENCODER MEASUREMENTS
        ################################################################################

        # Output matrix        
        Cd = np.zeros((q, n))

        if self.euler_angle_flag:
            Cd[0, 12] = 1
            Cd[1, 13] = 1
        else:
            Cd[0, 13] = 1
            Cd[1, 14] = 1

        ################################################################################

        self.state_corrector(y, Cd, Dd, Rd)

    def publish_actuator(self):

        current_time = self.node.get_clock().now()

        # Create Actuator message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        
        act.rudder = self.rudder_cmd * 180.0 / np.pi
        act.propeller = self.propeller_cmd * 60 * self.U_des / self.length

        # act.rudder = 35.0
        # act.propeller = 750.0

        self.actuator['pub'].publish(act)

        self.actuator_cmd = f'{act.rudder:.2f}, {act.propeller:.2f}'
    
    def publish_odometry(self):
        
        # Compute guidance and contorl
        if self.gnc_flag == "gnc":
            self.guidance()
            self.control()

        # Predict EKF state estimate
        self.state_predictor()

        current_time = self.node.get_clock().now()

        # Create Actuator message
        odo = Odometry()
        odo.header.stamp = current_time.to_msg()
        odo.header.frame_id = 'NED'
        odo.child_frame_id = 'BODY'

        odo.pose.pose.position = Point(x=self.x_hat[6] * self.length, y=self.x_hat[7] * self.length, z=self.x_hat[8] * self.length)
        
        if self.euler_angle_flag:
            quat = kin.eul_to_quat(self.x_hat[9:12])
            odo.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        else:
            odo.pose.pose.orientation = Quaternion(x=self.x_hat[10], y=self.x_hat[11], z=self.x_hat[12], w=self.x_hat[9])

        odo.twist.twist.linear = Vector3(x=self.x_hat[0] * self.U_des, y=self.x_hat[1] * self.U_des, z=self.x_hat[2] * self.U_des)
        odo.twist.twist.angular = Vector3(x=self.x_hat[3] * self.U_des / self.length, y=self.x_hat[4] * self.U_des / self.length, z=self.x_hat[5] * self.U_des / self.length)

        if self.euler_angle_flag:
            Ceul = self.P_hat[9:12][:, 9:12]
        else:
            G = kin.deul_dquat(self.x_hat[9:13])
            Cq = self.P_hat[9:13][:, 9:13]
            Ceul = G @ Cq @ G.T

        pose_cov = np.zeros((6,6), dtype=np.float64)
        pose_cov[0:3][:, 0:3] = self.P_hat[6:9][:, 6:9] * (self.length ** 2)
        pose_cov[3:6][:, 3:6] = Ceul
        odo.pose.covariance = pose_cov.flatten()

        twist_cov = np.zeros((6,6))
        twist_cov[0:3][:, 0:3] = self.P_hat[0:3][:, 0:3] * (self.U_des ** 2)
        twist_cov[3:6][:, 3:6] = self.P_hat[3:6][:, 3:6] * ((self.U_des / self.length) ** 2)
        odo.twist.covariance = twist_cov.flatten()

        self.odometry['pub'].publish(odo)

        if not self.terminate_flag:
            dt = (1 / self.rate) * self.U_des / self.length
            # self.score = self.score + np.abs(self.y_p_e) * dt + np.abs(self.x_hat[13] / (35 * np.pi / 180)) * dt
            self.current_time += dt

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
        print(f"Goal Waypoint (m)        : {self.goal_waypoint[0]:.2f}, {self.goal_waypoint[1]:.2f}, {self.goal_waypoint[2]:.2f} ")
        print(f"Distance to Goal (m)     : {np.linalg.norm(self.goal_waypoint - self.x_hat[6:9] * self.length):.2f}")
        # print(f"Score                    : {100 - self.score:.2f}")

        # print(f"Covariance               : {np.sqrt(self.P_hat[0, 0]):.2f}, {np.sqrt(self.P_hat[1,1]):.2f}, {np.sqrt(self.P_hat[2,2]):.2f}")
        # print(f"                         : {np.sqrt(self.P_hat[3,3]):.2f}, {np.sqrt(self.P_hat[4,4]):.2f}, {np.sqrt(self.P_hat[5,5]):.2f}")
        # print(f"                         : {np.sqrt(self.P_hat[6,6]):.2f}, {np.sqrt(self.P_hat[7,7]):.2f}, {np.sqrt(self.P_hat[8,8]):.2f}")
        # print(f"                         : {np.sqrt(self.P_hat[9,9]):.2f}, {np.sqrt(self.P_hat[10,10]):.2f}, {np.sqrt(self.P_hat[11,11]):.2f}, {np.sqrt(self.P_hat[12,12]):.2f}")
        # print(f"                         : {np.sqrt(self.P_hat[13,13]):.2f}, {np.sqrt(self.P_hat[14,14]):.2f}")
        print(f'\n\n')
    
    # Extended Kalman Filter state correction based on measurements

    def state_corrector(self, y, Cd, Dd, Rd, y_hat=None, sensor=None):
        
        if self.euler_angle_flag:
            if self.kinematic_kf_flag:
                n = 17
            else:
                n = 14
        else:
            if self.kinematic_kf_flag:
                n = 18
            else:
                n = 15
        try:
            
            #########################################################################################
            # WRITE YOUR CODE HERE TO UPDATE THE STATE AFTER MEASUREMENT
            #########################################################################################
            K = self.P_hat @ Cd.T @ np.linalg.inv(Cd @ self.P_hat @ Cd.T + Rd)

            if y_hat is None:
                innovation = (y - Cd @ self.x_hat - Dd @ self.u_cmd)
            else:
                innovation = (y - y_hat)
            
            # Implement SSA if IMU sensor and euler angle flag is True
            if sensor == 'IMU' and self.euler_angle_flag:
                for i in range(3):
                    innovation[i + 3] = kin.ssa(innovation[i + 3]) 
                
            x_hat_corr = K @ innovation
            
            if not self.euler_angle_flag:
                old_quat = self.x_hat[9:13]

            #########################################################################################

            # DO NOT EDIT THE FOLLOWING LINES (THIS IS NEEDED TO KEEP THE UPDATES STABLE)

            if self.kinematic_kf_flag:
                if self.euler_angle_flag:
                    thresh = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 10, 10, 10, 0.1, 0.1, 0.1, 0.6, 40, np.inf, np.inf, np.inf])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
                else:
                    thresh = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 10, 10, 10, np.inf, np.inf, np.inf, np.inf, 0.6, 40, np.inf, np.inf, np.inf])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
            else:
                if self.euler_angle_flag:
                    thresh = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 10, 10, 10, 0.1, 0.1, 0.1, 0.6, 40])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
                else:
                    thresh = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 10, 10, 10, np.inf, np.inf, np.inf, np.inf, 0.6, 40])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
            
            x_hat_corr = np.where(np.abs(x_hat_corr) > thresh, np.sign(x_hat_corr) * thresh, x_hat_corr)            
            self.x_hat = self.x_hat + x_hat_corr
            
            if not self.euler_angle_flag:
                self.x_hat[9:13] = self.x_hat[9:13] / np.linalg.norm(self.x_hat[9:13])

                quat_diff = kin.quat_multiply(old_quat, kin.quat_conjugate(self.x_hat[9:13]))
                if quat_diff[0] < 0:
                    self.x_hat[9:13] = -self.x_hat[9:13]
            
            if self.euler_angle_flag:
                rud_indx = 12
                prop_indx = 13
            else:
                rud_indx = 13
                prop_indx = 14
            
            self.x_hat[rud_indx] = kin.clip(self.x_hat[rud_indx], 35 * np.pi / 180)
            self.x_hat[prop_indx] = kin.clip(self.x_hat[prop_indx], 800.0 * self.length / (self.U_des * 60.0))

            self.P_hat = (np.eye(n) - K @ Cd) @ self.P_hat @ (np.eye(n) - K @ Cd).T + K @ Rd @ K.T
            
        except Exception as e:
            print('Corrector diverges: ', e)
            pass

    # Extended Kalman Filter state prediction based on model

    def state_predictor(self):
        
        if self.kinematic_kf_flag:
            if self.euler_angle_flag:
                n = 17; p = 2
            else:
                n = 18; p = 2
        else:
            if self.euler_angle_flag:
                n = 14; p = 2
            else:
                n = 15; p = 2
        

        if self.euler_angle_flag:
            rud_indx = 12
            prop_indx = 13
        else:
            rud_indx = 13
            prop_indx = 14
        
        h = 1/self.rate * (self.U_des / self.length)

        if self.kinematic_kf_flag:

            T_rud = self.vessel_data['T_rud']
            T_prop = self.vessel_data['T_prop']

            v_nb_b = self.x_hat[0:3]
            w_nb_b = self.x_hat[3:6]
            r_nb_n = self.x_hat[6:9]
            a_nb_b = self.x_hat[-3:]
            
            if self.euler_angle_flag:
                Theta_nb = self.x_hat[9:12]
                q_b_n = kin.eul_to_quat(Theta_nb)
                R_b_n = kin.eul_to_rotm(Theta_nb)
            else:
                q_b_n = self.x_hat[9:13]
                Theta_nb = kin.quat_to_eul(q_b_n)
                R_b_n = kin.quat_to_rotm(q_b_n)
            
            Amat = np.zeros((n,n))
            Bmat = np.zeros((n, p))

            Amat[0:3][:, 0:3] = -kin.Smat(w_nb_b)
            Amat[0:3][:, 3:6] = kin.Smat(v_nb_b)
            Amat[0:3][:, -3:] = np.eye(3)
            Amat[6:9][:, 0:3] = R_b_n
            Amat[rud_indx, rud_indx] = -1/T_rud
            Amat[prop_indx, prop_indx] = -1/T_prop
            
            if self.euler_angle_flag:
                Amat[6:9][:, 9:12] = kkf.d_by_dTheta_nb_R_b_n_v_nb_b(Theta_nb, v_nb_b)
                Amat[9:12][:, 3:6] = kin.eul_rate_matrix(Theta_nb)
                Amat[9:12][:, 9:12] = kkf.d_by_dTheta_nb_J2_w_nb_b(Theta_nb, w_nb_b)
                Amat[-3:][:, 3:6] = kin.Smat(a_nb_b)
                Amat[-3:][:, -3:] = -kin.Smat(w_nb_b)
            else:
                Amat[6:9][:, 9:13] = kkf.d_by_dq_b_n_R_b_n_v_nb_b(q_b_n, v_nb_b)
                Amat[9:13][:, 3:6] = kin.quat_rate_matrix(q_b_n)
                Amat[9:13][:, 9:13] = kkf.d_by_dq_b_n_J2_w_nb_b(q_b_n, w_nb_b)
                Amat[-3:][:, 3:6] = kin.Smat(a_nb_b)
                Amat[-3:][:, -3:] = -kin.Smat(w_nb_b)
            

            Bmat = np.zeros((n, p))
            Bmat[rud_indx, 0] = 1/T_rud
            Bmat[prop_indx, 1] = 1/T_prop

            Ad = scipy.linalg.expm(Amat * h)
            
            try:
                Bd = np.linalg.inv(Ad) @ (Ad - np.eye(n)) @ Bmat
            except:
                Bd = Bmat
            
            Ed = np.zeros((n, 6))
            Ed[3:6][:, 0:3] = R_b_n
            Ed[-3:][:, -3:] = R_b_n
            
            sig_process = np.array([1, 1, 1, 10, 10, 10]) * 1e-4
            Qd = np.diag(sig_process ** 2)
            
        else:

            u = self.x_hat[0]
            v = self.x_hat[1]
            r = self.x_hat[5]
            
            # Mass matrix
            M_RB = self.vessel_data['M_RB']
            M_A = self.vessel_data['M_A']
            D_l = self.vessel_data['Dl']
            K = self.vessel_data['K']
            M_inv = self.vessel_data['M_inv']

            # Cross Flow Drag coefficients
            Xuu = self.vessel_data['X_u_au']
            Yvv = self.vessel_data['Y_v_av']
            Yvr = self.vessel_data['Y_v_ar']
            Yrv = self.vessel_data['Y_r_av']
            Yrr = self.vessel_data['Y_r_ar']
            Nvv = self.vessel_data['N_v_av']
            Nvr = self.vessel_data['N_v_ar']
            Nrv = self.vessel_data['N_r_av']
            Nrr = self.vessel_data['N_r_ar']

            # Rudder and Propeller coefficients
            Yd = self.vessel_data['Y_d']
            Nd = self.vessel_data['N_d']
            # Xn = self.vessel_data['X_n']

            T_rud = self.vessel_data['T_rud']
            T_prop = self.vessel_data['T_prop']

            M = M_RB + M_A
            v_vec = self.x_hat[0:6]

            Nmat, F_damp = self.N_matrix(v_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)

            dF_damp_dv = np.zeros((6,6))
            for i in range(6):
                v2_vec = v_vec
                v1_vec = v_vec

                dx = 0.001

                v2_vec[i] = v2_vec[i] + dx
                v1_vec[i] = v1_vec[i] - dx

                _, F2_damp = self.N_matrix(v2_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)
                _, F1_damp = self.N_matrix(v1_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr)

                dF_damp_dv[:, i] = (F2_damp - F1_damp) / (2 * dx)

            if not self.euler_angle_flag:
                deul_dq = kin.deul_dquat(self.x_hat[9:13])
                H_mat = np.zeros((6, 7))
                H_mat[0:3][:, 0:3] = np.eye(3)
                H_mat[3:6][:, 3:7] = deul_dq

            Amat = np.zeros((n, n))

            if self.euler_angle_flag:
                Amat[0:6][:, 0:6] = - M_inv @ dF_damp_dv
                Amat[0:6][:, 6:12] = - M_inv @ K
                Amat[6:9][:, 0:3] = kin.eul_to_rotm(self.x_hat[9:12])
                Amat[9:12][:, 3:6] = kin.eul_rate_matrix(self.x_hat[9:12])
            else:
                Amat[0:6][:, 0:6] = - M_inv @ dF_damp_dv
                Amat[0:6][:, 6:13] = - M_inv @ K @ H_mat
                Amat[6:9][:, 0:3] = kin.quat_to_rotm(self.x_hat[9:13])
                Amat[9:13][:, 3:6] = kin.quat_rate_matrix(self.x_hat[9:13])
            
            wp = self.vessel_data['wp']
            tp = self.vessel_data['tp']
            D_prop = self.vessel_data['D_prop']
            pow_coeff = self.vessel_data['pow_coeff']

            Xn = 2 * (1 - tp) * (D_prop ** 2) * ((1 - wp) * D_prop * pow_coeff[1] + 2 * self.x_hat[prop_indx] * (D_prop ** 2) * pow_coeff[2])
            
            Amat[0:6][:, rud_indx:rud_indx+2] = M_inv @ np.array([
                [0, Xn],
                [Yd, 0],
                [0, 0],
                [0, 0],
                [0, 0],
                [Nd, 0]
            ])
            
            Amat[rud_indx, rud_indx] = -1/T_rud
            Amat[prop_indx, prop_indx] = -1/T_prop

            Bmat = np.zeros((n, p))
            Bmat[rud_indx, 0] = 1/T_rud
            Bmat[prop_indx, 1] = 1/T_prop

            Ad = scipy.linalg.expm(Amat * h)
            
            try:
                Bd = np.linalg.inv(Ad) @ (Ad - np.eye(n)) @ Bmat
            except:
                Bd = Bmat
            
            Ed = np.zeros((n, 6))
            Ed[0:6][:, 0:6] = M_inv

            sig_process = np.array([1, 1, 1e-2, 1e-2, 1e-2, 1]) * 1e-4
            Qd = np.diag(sig_process ** 2)

        try:
            if self.kinematic_kf_flag:
                
                # new_state = Ad @ self.x_hat + Bd @ self.u_cmd

                xd_hat1 = kkf.kinematic_ode(0, self.x_hat, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat2 = kkf.kinematic_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat1, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat3 = kkf.kinematic_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat2, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat4 = kkf.kinematic_ode(h, self.x_hat + h * xd_hat3, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)

                new_state = self.x_hat + (h / 6) * (xd_hat1 + 2 * xd_hat2 + 2 * xd_hat3 + xd_hat4)

            else:
                xd_hat1 = self.vessel_ode(0, self.x_hat, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat2 = self.vessel_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat1, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat3 = self.vessel_ode(0.5 * h, self.x_hat + 0.5 * h * xd_hat2, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)
                xd_hat4 = self.vessel_ode(h, self.x_hat + h * xd_hat3, self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag)

                new_state = self.x_hat + (h / 6) * (xd_hat1 + 2 * xd_hat2 + 2 * xd_hat3 + xd_hat4)

            # sol = solve_ivp(self.vessel_ode, (0, h), self.x_hat, args=(self.u_cmd[0], self.u_cmd[1], self.vessel_data, euler_angle_flag=self.euler_angle_flag))
            # new_state = np.array(sol.y)[:,-1]
            
            if np.any(np.isnan(new_state)):
                raise ValueError('NaN found in new state')
            
            if not self.euler_angle_flag:
                old_quat = self.x_hat[9:13]

            diff = new_state - self.x_hat

            if self.euler_angle_flag:
                
                for i in range(3):
                    diff[9 + i] = kin.ssa(diff[9 + i])

                if self.kinematic_kf_flag:
                    thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, 0.1, 0.1, 0.1, 0.6, 40, np.inf, np.inf, np.inf])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
                else:
                    thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, 0.1, 0.1, 0.1, 0.6, 40])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
            else:
                if self.kinematic_kf_flag:
                    thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, np.inf, np.inf, np.inf, np.inf, 0.6, 40, np.inf, np.inf, np.inf])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
                else:
                    thresh = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1, 1, 1, np.inf, np.inf, np.inf, np.inf, 0.6, 40])
                    # thresh = np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
            
            new_state = np.where(np.abs(diff) > thresh, self.x_hat + np.sign(diff) * thresh, new_state)
            self.x_hat = new_state
            
            if not self.euler_angle_flag:
                self.x_hat[9:13] = self.x_hat[9:13] / np.linalg.norm(self.x_hat[9:13])
                
                quat_diff = kin.quat_multiply(old_quat, kin.quat_conjugate(self.x_hat[9:13]))
                if quat_diff[0] < 0:
                    self.x_hat[9:13] = -self.x_hat[9:13]
            
            self.x_hat[rud_indx] = kin.clip(self.x_hat[rud_indx], 35 * np.pi / 180)
            self.x_hat[prop_indx] = kin.clip(self.x_hat[prop_indx], 800.0 * self.length / (self.U_des * 60.0))
            
            self.P_hat = Ad @ self.P_hat @ Ad.T + Ed @ Qd @ Ed.T
            
            pass
        except Exception as e:
            print('Predictor Diverges: ', e)
            pass        

    def N_matrix(self, v_vec, M, D_l, Xuu, Yvv, Yvr, Yrv, Yrr, Nvv, Nvr, Nrv, Nrr):

            u = v_vec[0]
            v = v_vec[1]
            r = v_vec[5]

            v1 = v_vec[0:3]
            v2 = v_vec[3:6]

            D_nl = -np.array([
                [Xuu * np.abs(u), 0, 0, 0, 0, 0],
                [0, Yvv * np.abs(v) + Yvr * np.abs(r), 0, 0, 0, Yrv * np.abs(v) + Yrr * np.abs(r)],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, Nvv * np.abs(v) + Nvr * np.abs(r), 0, 0, 0, Nrv * np.abs(v) + Nrr * np.abs(r)],
            ])
            
            M11 = M[0:3][:, 0:3]
            M12 = M[0:3][:, 3:6]
            M21 = M[3:6][:, 0:3]
            M22 = M[3:6][:, 3:6]

            C = np.zeros((6, 6))
            C[0:3][:, 3:6] = -kin.Smat(M11 @ v1 + M12 @ v2)
            C[3:6][:, 0:3] = -kin.Smat(M11 @ v1 + M12 @ v2)
            C[3:6][:, 3:6] = -kin.Smat(M21 @ v1 + M22 @ v2)

            N_mat = C + D_l + D_nl

            return N_mat, N_mat @ v_vec

    def guidance(self):
        # self.terminate_flag = True
        
        # Waypoint switching
        if np.linalg.norm(self.x_hat[6:9] - self.goal_waypoint / self.length) < 3 and self.terminate_flag is False:
            
            self.waypoint_index += 1

            if self.waypoint_index == self.waypoints.shape[0] - 1:
                self.terminate_flag = True

            else:                
                self.current_waypoint = self.waypoints[self.waypoint_index]
                self.goal_waypoint = self.waypoints[self.waypoint_index + 1]

        # Guidance mechanism
        pi_p = np.arctan2(self.goal_waypoint[1] - self.current_waypoint[1], 
                        self.goal_waypoint[0] - self.current_waypoint[0])

        R_n_p = np.array([[np.cos(pi_p), -np.sin(pi_p)], [np.sin(pi_p), np.cos(pi_p)]])
        
        x_n_i = self.current_waypoint[0:2] / self.length
        x_g_i = self.goal_waypoint[0:2] / self.length
        x_n = self.x_hat[6:8]

        # Along track distance of goal
        xy_g_e = R_n_p.T @ (x_g_i - x_n_i)
        x_g_e = xy_g_e[0]

        # Along and cross track distance of ship
        xy_p_e = R_n_p.T @ (x_n - x_n_i)
        self.x_p_e = xy_p_e[0]
        self.y_p_e = xy_p_e[1]

        # Look ahead distance
        Delta = 2
        kappa = 0.05

        dt = (1 / self.rate) * self.U_des / self.length

        #####################################################################################
        # WRITE YOUR CODE HERE TO DETERMINE THE DESIRED HEADING FROM ILOS GUIDANCE LAW
        #####################################################################################
        
        # Update the value of self.y_p_int (for ILOS guidance)
        yd_p_int = Delta * self.y_p_e / (Delta **2 + (self.y_p_e + kappa * self.y_p_int) ** 2)
        self.y_p_int = self.y_p_int + dt * yd_p_int

        # Compute the desired heading angle using self.y_p_e and self.y_p_int
        if self.x_p_e < x_g_e:
            self.psi_des = pi_p - np.arctan(self.y_p_e / Delta + kappa * self.y_p_int / Delta)
        else:
            self.psi_des = pi_p - np.pi + np.arctan(self.y_p_e / Delta + kappa * self.y_p_int / Delta)
        

        #####################################################################################

        # Calculating the u_err_int (for PI controller for propeller)
        U = np.linalg.norm(self.x_hat[0:2])
        ud_err_int = (1 - U)
        self.u_err_int = self.u_err_int + dt * ud_err_int
        

    def control(self):

        U = np.linalg.norm(self.x_hat[0:2])
        
        r = self.x_hat[5]

        if self.euler_angle_flag:
            eul = self.x_hat[9:12]
        else:
            eul = kin.quat_to_eul(self.x_hat[9:13])
        
        psi = eul[2]

        if not self.terminate_flag:
            
            #####################################################################################
            # WRITE YOUR CODE HERE FOR THE CONTROL LAW (HINT: REMEMBER SMALLEST SIGNED ANGLE)
            #####################################################################################

            Kp_R = 4
            Kd_R = 2

            rudder_cmd = 0.0
            rudder_cmd = Kp_R * kin.ssa(self.psi_des - psi) - Kd_R * r
            rudder_cmd = kin.clip(rudder_cmd, 35 * np.pi / 180)

            #####################################################################################

            # NO NEED TO TUNE THE PROPELLER CONTROL LAW
            Kp_P = 100
            Ki_P = 10

            propeller_cmd = Kp_P * (1 - U) + Ki_P * self.u_err_int
            propeller_cmd = kin.clip(propeller_cmd, 800.0 * self.length / (self.U_des * 60.0))

            self.rudder_cmd =  rudder_cmd
            self.propeller_cmd = propeller_cmd

        else:
            curr_time = self.node.get_clock().now()
            now_msg = curr_time.to_msg()  # Convert to a builtin_interfaces.msg.Time message
            secs = now_msg.sec
            nsecs = now_msg.nanosec
            tim = secs + nsecs / 1e9
            # print(f'{30 * np.sin(tim * 0.01):.2f}')
            self.rudder_cmd = 0.0 * np.pi / 180.0
            self.propeller_cmd = 0.0 * self.length / (self.U_des * 60.0)
        
        # Uncomment the following lines to only do localization without any control
        # Useful when trying to analyze data collected through RF mode
        
        if self.euler_angle_flag:
            rud_indx = 12
            prop_indx = 13
        else:
            rud_indx = 13
            prop_indx = 14
        
        # self.u_cmd = np.array([self.x_hat[rud_indx], self.x_hat[prop_indx]])

        # Uncomment the following line to do localization along with guidance and control
        # Useful when trying to analyze data collected in autonomous mode. 
        # This should be the default.

        self.u_cmd = np.array([self.rudder_cmd, self.propeller_cmd])

