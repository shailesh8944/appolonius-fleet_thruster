import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from interfaces.msg import Actuator
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from mav_simulator.module_kinematics import quat_to_eul, llh_to_ned, eul_to_quat, quat_to_rotm
from gnc.module_kin_kf import imu_mat, imu_model, gnss_mat, gnss_model, state_mats, plant_model
from gnc.module_kin_kf import encoder_mat, encoder_model

import numpy as np

class Navigation(Node):
    def __init__(self, vessel, ekf, llh0, th=None):
        super().__init__('navigation')
        self.vessel = vessel
        self.vessel_id = vessel.vessel_id
        self.vessel_name = vessel.vessel_name
        
        self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'
            
        # self.get_logger().info(f"Thresholds: {th}")
        
        self.ekf = ekf
        self.first_imu_flag = True
        self.first_pos_flag = True
        
        self.sensors = []
        
        # Calculate the number of actuators
        self.n_control_surfaces = vessel.n_control_surfaces if hasattr(vessel, 'n_control_surfaces') else 0
        self.n_thrusters = vessel.n_thrusters if hasattr(vessel, 'n_thrusters') else 0
        self.n_actuators = self.n_control_surfaces + self.n_thrusters
        
        # Dictionary to map actuator names to state indices
        self.actuator_map = {}
        
        # Calculate state indices for actuators
        if self.n_actuators > 0:
            # Control surfaces come first in the state vector
            for i in range(self.n_control_surfaces):
                actuator_name = f'cs_{i+1}'
                self.actuator_map[actuator_name] = 15 + i
                
            # Then thrusters
            for i in range(self.n_thrusters):
                actuator_name = f'th_{i+1}'
                self.actuator_map[actuator_name] = 15 + self.n_control_surfaces + i

        for sensor in self.vessel.vessel_config['sensors']['sensors']:
            
            self.sensors.append(sensor)
            
            if sensor['sensor_type'] == 'IMU':
                if 'sensor_topic' not in sensor:
                    # Use absolute topic path with namespace
                    sensor['sensor_topic'] = f'/{self.topic_prefix}/imu/data' if self.topic_prefix else '/imu/data'
                # Create a closure to capture the current sensor
                def create_imu_callback(sensor_config):
                    return lambda msg: self.imu_callback(msg, sensor_config)
                self.imu_sub = self.create_subscription(Imu, sensor['sensor_topic'], create_imu_callback(sensor), 10)
            elif sensor['sensor_type'] == 'GPS':
                if 'sensor_topic' not in sensor:
                    sensor['sensor_topic'] = f'/{self.topic_prefix}/gps' if self.topic_prefix else '/gps'
                # Create a closure to capture the current sensor
                def create_gnss_callback(sensor_config):
                    return lambda msg: self.gnss_callback(msg, sensor_config)
                self.gnss_sub = self.create_subscription(NavSatFix, sensor['sensor_topic'], create_gnss_callback(sensor), 10)
            elif sensor['sensor_type'] == 'UWB':
                if 'sensor_topic' not in sensor:
                    sensor['sensor_topic'] = f'/{self.topic_prefix}/uwb' if self.topic_prefix else '/uwb'
                # Create a closure to capture the current sensor
                def create_uwb_callback(sensor_config):
                    return lambda msg: self.uwb_callback(msg, sensor_config)
                self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, sensor['sensor_topic'], create_uwb_callback(sensor), 10)
            elif sensor['sensor_type'] == 'encoder':
                if 'sensor_topic' not in sensor:
                    actuator_type = sensor['actuator_type']
                    actuator_id = sensor['actuator_id']
                    sensor['sensor_topic'] = f'/{self.topic_prefix}/{actuator_type}_{actuator_id}/encoder' if self.topic_prefix else f'/{actuator_type}_{actuator_id}/encoder'
                # Create a closure to capture the current sensor
                def create_encoder_callback(sensor_config):
                    return lambda msg: self.encoder_callback(msg, sensor_config)
                self.encoder_sub = self.create_subscription(Actuator, sensor['sensor_topic'], create_encoder_callback(sensor), 10)
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 
                                             f'/{self.topic_prefix}/odometry' if self.topic_prefix else '/odometry', 
                                             10)
        
        # Publisher for all EKF states
        self.ekf_states_pub = self.create_publisher(Float64MultiArray, 
                                                   f'/{self.topic_prefix}/vessel_states_ekf' if self.topic_prefix else '/vessel_states_ekf', 
                                                   10)

        self.create_timer(self.vessel.dt, self.update_odometry)

        self.llh0 = llh0
        self.dt = self.vessel.dt
        self.th = th
    
    def imu_callback(self, msg, sensor):

        imu_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        imu_eul = quat_to_eul(imu_quat)
        imu_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        imu_omg = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        imu_acc = quat_to_rotm(imu_quat) @ imu_acc - np.array([0, 0, -9.81])
        
        y_imu = np.concatenate((imu_eul, imu_acc, imu_omg))[:, np.newaxis]
        
        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            # self.get_logger().info(f"{sensor['custom_covariance']}")
            R_imu = np.zeros((9, 9))
            R_imu[0:3, 0:3] = np.array(sensor['custom_covariance']['orientation_covariance']).reshape(3, 3)
            R_imu[3:6, 3:6] = np.array(sensor['custom_covariance']['angular_velocity_covariance']).reshape(3, 3)
            R_imu[6:9, 6:9] = np.array(sensor['custom_covariance']['linear_acceleration_covariance']).reshape(3, 3)
        else:
            R_imu = np.zeros((9, 9))
            R_imu[0:3, 0:3] = np.array(msg.orientation_covariance).reshape(3, 3)
            R_imu[3:6, 3:6] = np.array(msg.angular_velocity_covariance).reshape(3, 3)
            R_imu[6:9, 6:9] = np.array(msg.linear_acceleration_covariance).reshape(3, 3)
        
        Cd_imu = imu_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)
        

        if self.first_pos_flag or self.first_imu_flag:            
            self.ekf.x[3:6] = y_imu[0:3]
            self.ekf.x[9:12] = y_imu[3:6]
            self.ekf.x[12:15] = y_imu[6:9]
            
            if self.first_imu_flag:
                # self.get_logger().info(f"\nFirst IMU Measurement Obtained: {self.ekf.x[:, 0]} \n\n")
                self.first_imu_flag = False
        else:
            self.ekf.correct(
                y_imu, 
                Cd_imu, 
                R_imu, 
                meas_model=lambda x: imu_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th,
                imu_ssa=True
            )            
            # self.get_logger().info(f"IMU correction state: {self.ekf.x[:, 0]} \n")
        
    def gnss_callback(self, msg, sensor):
        
        gnss_pos = np.array([msg.latitude, msg.longitude, msg.altitude])
        y_gnss = llh_to_ned(gnss_pos, self.llh0)[:, np.newaxis]

        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            R_gnss = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
        else:
            R_gnss = np.array(msg.position_covariance).reshape(3, 3)

        Cd_gnss = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag or self.first_imu_flag:
            self.ekf.x[0:3] = y_gnss
            self.first_pos_flag = False
            # self.get_logger().info("First GNSS Measurement Obtained")
        else:
            self.ekf.correct(
                y_gnss, 
                Cd_gnss, 
                R_gnss, 
                meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th,
                imu_ssa=False
            )
        
    def uwb_callback(self, msg, sensor):
        y_uwb = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])[:, np.newaxis]

        r_bs_b = np.array(sensor['sensor_location'])
        Theta_bs = np.array(sensor['sensor_orientation'])

        if sensor['use_custom_covariance']:
            # self.get_logger().info(f"{sensor['custom_covariance']}")
            R_uwb = np.array(sensor['custom_covariance']['position_covariance']).reshape(3, 3)
        else:
            R_uwb = np.zeros((3, 3))
            R_uwb[0:3, 0:3] = np.array(msg.pose.covariance).reshape(6, 6)[0:3, 0:3]

        Cd_uwb = gnss_mat(self.ekf.x[:, 0], r_bs_b=r_bs_b, Theta_bs=Theta_bs)

        if self.first_pos_flag or self.first_imu_flag:
            self.ekf.x[0:3] = y_uwb

            if self.first_pos_flag:
                # self.get_logger().info(f"\nFirst UWB Measurement Obtained: {self.ekf.x[:, 0]} \n\n")
                self.first_pos_flag = False
        else:
            self.ekf.correct(
                y_uwb, 
                Cd_uwb, 
                R_uwb, 
                meas_model=lambda x: gnss_model(x, r_bs_b=r_bs_b, Theta_bs=Theta_bs), 
                threshold=self.th,
                imu_ssa=False
            )
            # self.get_logger().info(f"UWB correction state: {self.ekf.x[:, 0]} \n")
    
    def encoder_callback(self, msg, sensor):
        """Process encoder measurements for actuator positions"""
        
        # Only process if we have actuators defined
        if self.n_actuators == 0:
            return
            
        # Find the actuator values and names in the message
        actuator_values = msg.actuator_values
        actuator_names = msg.actuator_names
        
        # Extract covariance values
        covariance = np.array(msg.covariance)
        if len(covariance) != len(actuator_values):
            # If not enough covariance values, use defaults
            covariance = np.ones(len(actuator_values)) * 0.01
        
        # Process each actuator in the message
        for i, name in enumerate(actuator_names):
            if name in self.actuator_map:
                # Get the state index for this actuator
                idx = self.actuator_map[name]
                
                # Create measurement vector and covariance for this actuator
                y_encoder = np.array([actuator_values[i]])[:, np.newaxis]
                R_encoder = np.array([[covariance[i]]])
                
                # Get the Cd matrix for this encoder
                Cd_encoder = encoder_mat(self.ekf.x[:, 0], actuator_idx=idx)
                
                # Update the EKF with the measurement
                if not self.first_imu_flag:
                    # self.get_logger().info(f"Setting actuator state: {y_encoder} for actuator: {name}, idx: {idx}")
                    self.ekf.correct(
                        y_encoder,
                        Cd_encoder,
                        R_encoder,
                        meas_model=lambda x: encoder_model(x, actuator_idx=idx),
                        threshold=self.th,
                        imu_ssa=False
                    )
                else:
                    # During initialization, set the actuator state directly
                    # self.get_logger().info(f"Setting actuator state: {y_encoder} for actuator: {name}, idx: {idx}")
                    self.ekf.x[idx] = y_encoder
            else:
                self.get_logger().warning(f"Received encoder measurement for unknown actuator: {name}")

    def update_odometry(self):

        Amat, Emat = state_mats(self.ekf.x.flatten())
        self.ekf.predict(
            u=np.array([[0]]), 
            A=Amat, 
            B=None, 
            E=Emat,
            plant_model=plant_model,
            discrete_flag=False,
            threshold=self.th
        )

        # self.get_logger().info(f"Time: {self.ekf.t:.2f}, Surge: {self.ekf.x[0,0]:.2f} m, Sway: {self.ekf.x[1,0]:.2f} m, Heave: {self.ekf.x[2,0]:.2f} m, Roll: {self.ekf.x[3,0]*180/np.pi:.2f} deg, Pitch: {self.ekf.x[4,0]*180/np.pi:.2f} deg, Yaw: {self.ekf.x[5,0]*180/np.pi:.2f} deg")

        # Publish odometry message
        self.publish_odometry()
        
        # Publish all EKF states
        self.publish_ekf_states()
    
    def publish_odometry(self):
        """Publish the vessel odometry message."""
        odom_msg = Odometry()
        
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'NED'
        odom_msg.child_frame_id = 'BODY'
        
        odom_msg.pose.pose.position.x = self.ekf.x[0,0]
        odom_msg.pose.pose.position.y = self.ekf.x[1,0]
        odom_msg.pose.pose.position.z = self.ekf.x[2,0]

        quat = eul_to_quat(np.array([self.ekf.x[3,0], self.ekf.x[4,0], self.ekf.x[5,0]]))
        odom_msg.pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

        pose_cov = np.zeros(36).reshape(6, 6)
        pose_cov[0:3, 0:3] = self.ekf.P[0:3, 0:3]
        pose_cov[3:6, 3:6] = self.ekf.P[3:6, 3:6]
        odom_msg.pose.covariance = pose_cov.flatten()
        
        odom_msg.twist.twist.linear.x = self.ekf.x[6,0]
        odom_msg.twist.twist.linear.y = self.ekf.x[7,0]
        odom_msg.twist.twist.linear.z = self.ekf.x[8,0]
        
        odom_msg.twist.twist.angular.x = self.ekf.x[9,0]
        odom_msg.twist.twist.angular.y = self.ekf.x[10,0]
        odom_msg.twist.twist.angular.z = self.ekf.x[11,0]

        twist_cov = np.zeros(36).reshape(6, 6)
        twist_cov[0:3, 0:3] = self.ekf.P[6:9, 6:9]
        twist_cov[3:6, 3:6] = self.ekf.P[9:12, 9:12]
        odom_msg.twist.covariance = twist_cov.flatten()

        self.odom_pub.publish(odom_msg)
    
    def publish_ekf_states(self):
        """Publish all EKF states in a Float64MultiArray message."""
        # Create message
        msg = Float64MultiArray()
        
        # Get the current EKF state vector (flattening the column vector)
        states = self.ekf.x.flatten()
        
        # Add metadata for understanding the message structure
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "states"
        msg.layout.dim[0].size = len(states)
        msg.layout.dim[0].stride = 1
        
        # Set data field with all state values
        msg.data = states.tolist()
        
        # Publish the message
        self.ekf_states_pub.publish(msg)
        