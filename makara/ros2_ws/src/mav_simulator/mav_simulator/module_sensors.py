"""
File: module_sensors.py
Description: This file implements various sensor models for the MAV simulator. It provides:
             
             - A BaseSensor class that defines the common interface for all sensors
             - Specific sensor implementations including:
               * IMU (Inertial Measurement Unit)
               * GPS (Global Positioning System)
               * UWB (Ultra-Wideband positioning)
               * Encoders (for measuring actuator positions)
               * DVL (Doppler Velocity Log)
               
             - Realistic sensor modeling with:
               * Appropriate noise models
               * Configurable update rates
               * Placement and orientation offsets
               * ROS2 message type conversion
               
             - A factory function for creating sensor instances from configuration
             
             This module enables realistic sensing capabilities for simulated vessels,
             allowing for the development and testing of perception and estimation algorithms.
             
Author: MAV Simulator Team
"""

import numpy as np
from rclpy.node import Node
from mav_simulator.terminalMessages import print_info
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped
# from interfaces.msg import Actuator
import mav_simulator.module_kinematics as kin

class BaseSensor:
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        self.vessel_id = vessel_id
        self.topic_prefix = topic_prefix
        self.sensor_type = sensor_config['sensor_type']
        self.rate = sensor_config['publish_rate']
        # Use the specified sensor topic if available
        self.topic = sensor_config.get('sensor_topic', None)
        self.id = sensor_config.get('id', 0)
        self.vessel_node = vessel_node
        self.use_custom_covariance = sensor_config.get('use_custom_covariance', False)
        self.custom_covariance = sensor_config.get('custom_covariance', {})

class IMUSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        self.orientation = np.array(sensor_config['sensor_orientation'])
        
        # Default noise parameters
        self.eul_rms = np.array([1, 1, 1]) * 1e-2
        self.eul_cov = np.diag(self.eul_rms ** 2)
        self.ang_vel_rms = np.array([1, 1, 1]) * 1e-2
        self.ang_vel_cov = np.diag(self.ang_vel_rms ** 2)
        self.lin_acc_rms = np.array([1, 1, 1]) * 1.5e-1
        self.lin_acc_cov = np.diag(self.lin_acc_rms ** 2)
        
        # Override with custom covariance if provided
        if self.use_custom_covariance and self.custom_covariance:
            # for cov_item in self.custom_covariance:
            if 'orientation_covariance' in self.custom_covariance:
                self.eul_cov = np.array(self.custom_covariance['orientation_covariance']).reshape(3, 3)
            if 'angular_velocity_covariance' in self.custom_covariance:
                self.ang_vel_cov = np.array(self.custom_covariance['angular_velocity_covariance']).reshape(3, 3)
            if 'linear_acceleration_covariance' in self.custom_covariance:
                self.lin_acc_cov = np.array(self.custom_covariance['linear_acceleration_covariance']).reshape(3, 3)

    def get_measurement(self,quat=False):
        state = self.vessel_node.vessel.current_state
        state_der = self.vessel_node.vessel.current_state_der
        
        # Handle both quaternion and euler angle inputs
        if not quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            quat = kin.eul_to_quat(eul)
            orientation_eul = self.orientation
            orientation_quat = kin.eul_to_quat(orientation_eul)
        else:
            quat = state[9:13]
            orientation_quat = self.orientation

        omg_bcs = state[3:6]
        v_bcs = state[0:3]        
        alpha = state_der[3:6]

        q_sensor = kin.rotm_to_quat(kin.quat_to_rotm(quat) @ kin.quat_to_rotm(orientation_quat))
        q_sensor = kin.eul_to_quat(kin.quat_to_eul(q_sensor) + np.random.multivariate_normal(np.zeros(3), self.eul_cov))
        
        acc_bcs = state_der[0:3] + np.cross(omg_bcs, v_bcs)
        acc_s_bcs = acc_bcs + np.cross(alpha, self.location) + np.cross(omg_bcs, np.cross(omg_bcs, self.location))
        acc_sensor = kin.quat_to_rotm(orientation_quat).T @ acc_s_bcs
        
        acc_sensor = acc_sensor + kin.quat_to_rotm(q_sensor).T @ np.array([0, 0, -self.vessel_node.vessel.g])
        acc_sensor = acc_sensor + np.random.multivariate_normal(np.zeros(3), self.lin_acc_cov)


        omg_sensor = kin.quat_to_rotm(orientation_quat).T @ omg_bcs
        omg_sensor = omg_sensor + np.random.multivariate_normal(np.zeros(3), self.ang_vel_cov)

        return {
            'orientation': q_sensor,
            'angular_velocity': omg_sensor,
            'linear_acceleration': acc_sensor,
            'orientation_covariance': self.eul_cov.flatten(),
            'angular_velocity_covariance': self.ang_vel_cov.flatten(),
            'linear_acceleration_covariance': self.lin_acc_cov.flatten()
        }

class GPSSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Default noise parameters
        self.gps_rms = np.array([3, 3, 3], dtype=np.float64)
        self.gps_cov = np.diag(self.gps_rms ** 2)
        
        # Override with custom covariance if provided
        if self.use_custom_covariance and self.custom_covariance:
            # for cov_item in self.custom_covariance:
            if 'position_covariance' in self.custom_covariance:
                self.gps_cov = np.array(self.custom_covariance['position_covariance']).reshape(3, 3)

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        llh0 = self.vessel_node.vessel.gps_datum
        
        # Handle both quaternion and euler angle inputs
        if not quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            orientation = kin.eul_to_quat(eul)
        else:
            orientation = state[9:13]
            
        ned = state[6:9] + kin.quat_to_rotm(orientation) @ self.location
        ned = ned + np.random.multivariate_normal(np.zeros(3), self.gps_cov)
        llh = kin.ned_to_llh(ned, llh0)

        return {
            'latitude': llh[0],
            'longitude': llh[1],
            'altitude': llh[2],
            'position_covariance': self.gps_cov.flatten()
        }

class UWBSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        
        # Default noise parameters
        self.uwb_rms = np.array([1, 1, 1], dtype=np.float64)
        self.uwb_cov = np.diag(self.uwb_rms ** 2)
        self.uwb_cov_full = -np.eye(6)
        self.uwb_cov_full[0:3][:, 0:3] = self.uwb_cov
        
        # Override with custom covariance if provided
        if self.use_custom_covariance and self.custom_covariance:
            if 'position_covariance' in self.custom_covariance:
                self.uwb_cov = np.array(self.custom_covariance['position_covariance']).reshape(3, 3)
                self.uwb_cov_full[0:3][:, 0:3] = self.uwb_cov

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        ned = state[6:9]
        
        # Handle both quaternion and euler angle inputs
        if not quat:
            # Convert euler angles to quaternion
            eul = state[9:12]
            orientation = kin.eul_to_quat(eul)
        else:
            orientation = state[9:13]
            
        r_sen = ned + kin.quat_to_rotm(orientation) @ self.location
        r_sen = r_sen + np.random.multivariate_normal(np.zeros(3), self.uwb_cov)

        return {
            'position': r_sen,
            'covariance': self.uwb_cov_full.flatten()
        }

class EncoderSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)

        # Store specific actuator details
        self.actuator_type = sensor_config['actuator_type'] # e.g., 'Rudder', 'Thruster'
        self.actuator_id = sensor_config['actuator_id'] # 1-based ID

        # Determine the index in the state vector for this specific actuator
        vessel = vessel_node.vessel
        use_quaternion = vessel.use_quaternion
        attitude_size = 4 if use_quaternion else 3
        base_idx = 9 + attitude_size

        self.state_index = -1
        self.unit_conversion = 1.0 # Default: no conversion (e.g., RPM for thrusters)
        self.noise_rms = 0.0
        self.actuator_name_prefix = ''

        # Control Surfaces (assuming 'Rudder', 'Elevator', 'Aileron' map to control surfaces)
        if 'control_surfaces' in vessel.__dict__ and self.actuator_type in ['Rudder', 'Elevator', 'Aileron']: # Add other types if needed
            self.actuator_name_prefix = 'cs'
            num_control_surfaces = len(vessel.control_surfaces['control_surfaces'])
            if 1 <= self.actuator_id <= num_control_surfaces:
                self.state_index = base_idx + self.actuator_id - 1
                self.unit_conversion = 180.0 / np.pi # State is radians, output is degrees
                self.noise_rms = 1e-1 # 0.1 degrees RMS noise for control surfaces
            else:
                raise ValueError(f"Invalid actuator_id {self.actuator_id} for control surface type {self.actuator_type}")

        # Thrusters
        elif 'thrusters' in vessel.__dict__ and self.actuator_type == 'Thruster': # Assuming 'Thruster' type
             self.actuator_name_prefix = 'th'
             num_control_surfaces = len(vessel.control_surfaces.get('control_surfaces', [])) if 'control_surfaces' in vessel.__dict__ else 0
             num_thrusters = len(vessel.thrusters['thrusters'])
             if 1 <= self.actuator_id <= num_thrusters:
                 thruster_start_idx = base_idx + num_control_surfaces
                 self.state_index = thruster_start_idx + self.actuator_id - 1
                 self.unit_conversion = 60.0 # State is rad/s (assuming), output is RPM - check vessel model if state is different!
                 self.noise_rms = 1e-1 # 0.1 RPM RMS noise for thrusters - check unit consistency!
             else:
                 raise ValueError(f"Invalid actuator_id {self.actuator_id} for thruster type {self.actuator_type}")
        
        else:
             raise ValueError(f"Unknown or unsupported actuator_type: {self.actuator_type} or vessel missing required actuators.")

        if self.state_index == -1:
             raise ValueError(f"Could not determine state index for actuator {self.actuator_type} ID {self.actuator_id}")

        self.encoder_cov = self.noise_rms ** 2 # Variance for the single measured value

    def get_measurement(self):
        state = self.vessel_node.vessel.current_state
        
        # Get the specific actuator value from the state vector
        actuator_value_raw = state[self.state_index]
        
        # Apply unit conversion
        actuator_value_converted = actuator_value_raw * self.unit_conversion
        
        # Add noise to the converted measurement
        actuator_value_noisy = actuator_value_converted + np.random.normal(0, self.noise_rms)

        # Create actuator name string (e.g., 'cs_1', 'th_1')
        actuator_name = f'{self.actuator_name_prefix}_{self.actuator_id}'
        
        return {
            'actuator_value': actuator_value_noisy,
            'actuator_name': actuator_name,
            'covariance': [self.encoder_cov] # Covariance is just the variance for a single value
        }

class DVLSensor(BaseSensor):
    def __init__(self, sensor_config, vessel_id, topic_prefix, vessel_node):
        super().__init__(sensor_config, vessel_id, topic_prefix, vessel_node)
        self.location = np.array(sensor_config['sensor_location'])
        self.orientation = np.array(sensor_config['sensor_orientation'])
        
        # Default noise parameters - typical DVL has accuracy of ~0.2-1% of measured velocity
        self.vel_rms = np.array([0.05, 0.05, 0.05])  # 5cm/s RMS noise in each axis
        self.vel_cov = np.diag(self.vel_rms ** 2)
        
        # Override with custom covariance if provided
        if self.use_custom_covariance and self.custom_covariance:
            # for cov_item in self.custom_covariance:
            if 'linear_velocity_covariance' in self.custom_covariance:
                self.vel_cov = np.array(self.custom_covariance['linear_velocity_covariance']).reshape(3, 3)

    def get_measurement(self, quat=False):
        state = self.vessel_node.vessel.current_state
        
        # Get body-frame velocity and add noise
        v_body = state[0:3]
        v_body_noisy = v_body + np.random.multivariate_normal(np.zeros(3), self.vel_cov)
        
        return {
            'velocity': v_body_noisy,
            'covariance': self.vel_cov.flatten()  # Return 3x3 covariance matrix
        }

def create_sensor(sensor_config, vessel_id, topic_prefix, vessel_node):
    """Factory function to create appropriate sensor object based on sensor type"""
    sensor_type = sensor_config['sensor_type']
    sensor_classes = {
        'IMU': IMUSensor,
        'GPS': GPSSensor,
        'UWB': UWBSensor,
        'encoder': EncoderSensor,
        'DVL': DVLSensor
    }
    
    if sensor_type not in sensor_classes:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
        
    return sensor_classes[sensor_type](sensor_config, vessel_id, topic_prefix, vessel_node) 