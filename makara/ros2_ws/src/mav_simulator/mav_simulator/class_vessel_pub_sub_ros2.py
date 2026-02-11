"""
File: class_vessel_pub_sub_ros2.py
Description: This file implements the ROS2 communication interface for vessel objects in the
             MAV simulator. The Vessel_Pub_Sub class:
             
             - Creates and manages ROS2 publishers for vessel state and sensor data
             - Sets up subscribers for actuator commands and control inputs
             - Handles message conversion between simulator data structures and ROS2 messages
             - Manages sensor data publication at appropriate rates
             - Provides callback functions for processing incoming ROS2 messages
             - Bridges the gap between the simulation core and ROS2 ecosystem
             
             This class enables the integration of the MAV simulator with other ROS2 nodes,
             allowing for distributed simulation, external control systems, visualization,
             and data recording.
             
Author: MAV Simulator Team
"""

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, Image, LaserScan, NavSatFix, NavSatStatus
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, Twist, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from interfaces.msg import Actuator, DVL
import mav_simulator.module_kinematics as kin
from mav_simulator.module_sensors import create_sensor
from mav_simulator.terminalMessages import print_info

class Vessel_Pub_Sub():
    """ROS2 publisher/subscriber interface for a vessel.
    
    This class handles all ROS2 communication for a vessel, including:
    - Publishing sensor data
    - Publishing odometry
    - Subscribing to actuator commands
    """
    
    def __init__(self, vessel, world_node):
        """Initialize the vessel interface.
        
        Args:
            vessel: The vessel object this interface is for
            node: The ROS2 node to use for communication
        """
        self.world_node = world_node
        self.vessel = vessel
        self.vessel_id = vessel.vessel_id
        self.vessel_name = vessel.vessel_name
        self.topic_prefix = f'{self.vessel_name}_{self.vessel_id:02d}'
        
        # Get control surfaces and thrusters directly from vessel
        self.control_surface_ids = {}
        self.thruster_ids = {}
        self.control_surfaces = []
        self.thrusters = []

        if getattr(vessel, 'control_surfaces', None):
            self.control_surfaces = vessel.control_surfaces.get('control_surfaces', [])
            for idx, cs in enumerate(self.control_surfaces):
                cs_id = cs.get('control_surface_id', idx+1)
                self.control_surface_ids[f'cs_{cs_id}'] = idx

        if getattr(vessel, 'thrusters', None):
            self.thrusters = vessel.thrusters.get('thrusters', [])
            for idx, th in enumerate(self.thrusters):
                th_id = th.get('thruster_id', idx+1)
                # Primary ID
                self.thruster_ids[f'th_{th_id}'] = idx

                # Alias based on lateral placement if available
                try:
                    y_pos = th.get('thruster_location', [None, None, None])[1]
                except Exception:
                    y_pos = None

                if y_pos is not None:
                    if y_pos < 0:
                        for alias in ['th_port', 'th_port_thruster', 'th_portside']:
                            self.thruster_ids.setdefault(alias, idx)
                    elif y_pos > 0:
                        for alias in ['th_stbd', 'th_starboard', 'th_starboard_thruster']:
                            self.thruster_ids.setdefault(alias, idx)
                    else:
                        self.thruster_ids.setdefault('th_center', idx)

            # Ensure common port/stbd aliases exist even without y-location
            if len(self.thrusters) >= 2:
                self.thruster_ids.setdefault('th_port', 0)
                self.thruster_ids.setdefault('th_stbd', 1)
            elif len(self.thrusters) == 1:
                # Map both names to the single thruster to avoid callback errors
                self.thruster_ids.setdefault('th_port', 0)
                self.thruster_ids.setdefault('th_stbd', 0)
        
        # self.delta_c = np.zeros(len(self.control_surfaces))
        # self.n_c = np.zeros(len(self.thrusters))
        
        # Create sensors
        self.sensors = []
        if 'sensors' in vessel.vessel_config:
            for sensor_config in vessel.vessel_config['sensors'].get('sensors', []):
                # Use the sensor_topic from config if available, otherwise use default topic
                sensor_name = sensor_config.get('name', sensor_config['sensor_type'].lower())
                default_topic = f'{self.topic_prefix}/{sensor_name}'
                sensor_topic = sensor_config.get('sensor_topic', default_topic)
                
                # Update sensor_config with the topic
                sensor_config['topic'] = sensor_topic
                
                sensor = create_sensor(sensor_config, self.vessel_id, self.topic_prefix, self)
                self.sensors.append({
                    'sensor': sensor,
                    'pub': self.world_node.create_publisher(self._get_msg_type(sensor.sensor_type), sensor_topic, 10),
                    'timer': self.world_node.create_timer(1/sensor.rate, lambda s=sensor: self._publish_sensor(s))
                })
        
        # Create odometry publisher
        self.odometry = {
            'pub': self.world_node.create_publisher(Odometry, f'{self.topic_prefix}/odometry_sim', 10),
            'timer': self.world_node.create_timer(self.vessel.vessel_config['time_step'], self.publish_odometry)
        }
        
        # Create vessel state publisher
        self.vessel_state = {
            'pub': self.world_node.create_publisher(Float64MultiArray, f'{self.topic_prefix}/vessel_state', 10),
            'timer': self.world_node.create_timer(self.vessel.vessel_config['time_step'], self.publish_vessel_state)
        }
        
        # Create actuator subscribers
        self.actuator_sub = self.world_node.create_subscription(
            Actuator, 
            f'{self.topic_prefix}/actuator_cmd', 
            self.actuator_callback, 
            1
        )

    def _get_msg_type(self, sensor_type):
        """Get the ROS message type for a given sensor type."""
        msg_types = {
            'IMU': Imu,
            'GPS': NavSatFix,
            'UWB': PoseWithCovarianceStamped,
            'DVL': DVL,
            'encoder': Actuator
        }
        return msg_types.get(sensor_type)

    def _publish_sensor(self, sensor):
        """Publish sensor data.
        
        Args:
            sensor: The sensor object to publish data for
        """
        measurement = sensor.get_measurement()
        msg = self._create_sensor_message(sensor.sensor_type, measurement)
        
        # Find the publisher for this sensor
        for s in self.sensors:
            if s['sensor'] == sensor:
                s['pub'].publish(msg)
                break

    def _create_sensor_message(self, sensor_type, measurement):
        """Create a ROS message for sensor data.
        
        Args:
            sensor_type: Type of the sensor
            measurement: Dictionary containing sensor measurements
            
        Returns:
            The appropriate ROS message type filled with sensor data
        """
        current_time = self.world_node.get_clock().now().to_msg()
        
        if sensor_type == 'IMU':
            msg = Imu()
            msg.header.stamp = current_time
            msg.header.frame_id = f'{self.topic_prefix}_imu_frame'
            msg.orientation = Quaternion(
                x=measurement['orientation'][1],
                y=measurement['orientation'][2],
                z=measurement['orientation'][3],
                w=measurement['orientation'][0]
            )
            msg.angular_velocity = Vector3(
                x=measurement['angular_velocity'][0],
                y=measurement['angular_velocity'][1],
                z=measurement['angular_velocity'][2]
            )
            msg.linear_acceleration = Vector3(
                x=measurement['linear_acceleration'][0],
                y=measurement['linear_acceleration'][1],
                z=measurement['linear_acceleration'][2]
            )
            msg.orientation_covariance = measurement['orientation_covariance']
            msg.angular_velocity_covariance = measurement['angular_velocity_covariance']
            msg.linear_acceleration_covariance = measurement['linear_acceleration_covariance']
            
        elif sensor_type == 'GPS':
            msg = NavSatFix()
            msg.header.stamp = current_time
            msg.header.frame_id = 'ECEF'
            msg.status = NavSatStatus(status=0, service=1)
            msg.latitude = measurement['latitude']
            msg.longitude = measurement['longitude']
            msg.altitude = measurement['altitude']
            msg.position_covariance = measurement['position_covariance']
            
        elif sensor_type == 'UWB':
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = current_time
            msg.header.frame_id = 'NED'
            msg.pose.pose.position = Point(
                x=measurement['position'][0],
                y=measurement['position'][1],
                z=measurement['position'][2]
            )
            msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.pose.covariance = measurement['covariance']
            
        elif sensor_type == 'DVL':
            msg = DVL()
            msg.header.stamp = current_time
            msg.header.frame_id = f'{self.topic_prefix}_dvl_frame'
            msg.velocity = Vector3(
                x=measurement['velocity'][0],
                y=measurement['velocity'][1],
                z=measurement['velocity'][2]
            )
            msg.covariance = measurement['covariance']
            
        elif sensor_type == 'encoder':
            msg = Actuator()
            msg.header.stamp = current_time
            # Populate with single value and name from the specific encoder
            msg.actuator_values = [measurement['actuator_value']]
            msg.actuator_names = [measurement['actuator_name']]
            # The Actuator message in interfaces.msg does not seem to have a standard covariance field.
            # If you have a custom message with covariance, you can add:
            # msg.covariance = measurement['covariance'] 
            
        return msg

    def actuator_callback(self, msg):
        """Handle actuator command messages.
        
        The actuator values are received in their respective units:
        - Control surfaces (e.g., rudder): degrees, converted to radians (prefix: cs_)
        - Thrusters (e.g., propeller): RPM (prefix: th_)
        
        Expected format for actuator_names:
        - Control surfaces: 'cs_ID' (e.g., 'cs_1', 'cs_2')
        - Thrusters: 'th_ID' (e.g., 'th_1', 'th_2')
        """
        try:
            self.world_node.get_logger().info(f"Actuator message: {list(zip(msg.actuator_names, msg.actuator_values))}")
        except Exception:
            pass

        if len(msg.actuator_names) != len(msg.actuator_values):
            self.world_node.get_logger().warn('Mismatch between actuator IDs and values length')
            return
            
        for actuator_id, value in zip(msg.actuator_names, msg.actuator_values):
            try:
                if actuator_id.startswith('cs_'):
                    # Handle control surface
                    if actuator_id in self.control_surface_ids:
                        idx = self.control_surface_ids[actuator_id]
                        self.vessel.delta_c[idx] = value * np.pi / 180.0
                        # print_info(f"control_surface {actuator_id} idx: {idx}, value: {value} deg -> {self.vessel.delta_c[idx]} rad")
                    else:
                        self.world_node.get_logger().warn(f'Unknown control surface ID: {actuator_id}')
                        
                elif actuator_id.startswith('th_'):
                    # Handle thruster with alias support
                    idx = self.thruster_ids.get(actuator_id)
                    if idx is None:
                        # Try lowercase alias
                        idx = self.thruster_ids.get(actuator_id.lower())
                    if idx is None:
                        self.world_node.get_logger().warn(f'Unknown thruster ID: {actuator_id}')
                    else:
                        self.vessel.n_c[idx] = value
                        
                else:
                    self.world_node.get_logger().warn(f'Invalid actuator ID format: {actuator_id}. Must start with cs_ or th_')
            except (ValueError, IndexError):
                self.world_node.get_logger().warn(f'Unknown actuator ID: {actuator_id}')

    def publish_odometry(self):
        """Publish vessel odometry data."""
        current_time = self.world_node.get_clock().now()
        
        msg = Odometry()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'NED'
        msg.child_frame_id = 'BODY'
        
        state = self.vessel.current_state
        
        # Position and orientation
        msg.pose.pose.position = Point(x=state[6], y=state[7], z=state[8])
        
        if self.vessel.use_quaternion:
            msg.pose.pose.orientation = Quaternion(
                x=state[10], y=state[11], z=state[12], w=state[9]
            )
        else:
            state_quat = kin.eul_to_quat(state[9:12])
            msg.pose.pose.orientation = Quaternion(
                x=state_quat[1], y=state_quat[2], z=state_quat[3], w=state_quat[0]
            )
        
        # Linear and angular velocities
        msg.twist.twist.linear = Vector3(x=state[0], y=state[1], z=state[2])
        msg.twist.twist.angular = Vector3(x=state[3], y=state[4], z=state[5])
        
        # Zero covariance as this is simulation data
        msg.pose.covariance = np.zeros((6, 6)).flatten()
        msg.twist.covariance = np.zeros((6, 6)).flatten()
        
        self.odometry['pub'].publish(msg)

    def publish_vessel_state(self):
        """Publish complete vessel state including control surfaces and thrusters."""
        current_time = self.world_node.get_clock().now()
        
        msg = Float64MultiArray()
        
        # Get the full state vector
        state = self.vessel.current_state
        
        # Determine state vector structure based on quaternion vs euler
        use_quaternion = hasattr(self.vessel, 'use_quaternion') and self.vessel.use_quaternion
        attitude_size = 4 if use_quaternion else 3
        attitude_end = 9 + attitude_size
        
        # Calculate indices for control surfaces and thrusters
        control_start = attitude_end
        thruster_start = control_start + self.vessel.n_control_surfaces
        
        # Create label for describing the state vector contents
        state_description = (
            "State vector: [u, v, w, p, q, r, x, y, z, "
            f"{'q0, q1, q2, q3' if use_quaternion else 'phi, theta, psi'}, "
            "control_surfaces, thrusters]"
        )
        
        # Add description as a field in the message
        msg.data = state.tolist()
        
        # Log state description once (or infrequently)
        if hasattr(self, '_state_desc_logged') == False:
            self.world_node.get_logger().info(f"Publishing vessel state: {state_description}")
            self._state_desc_logged = True
            
        self.vessel_state['pub'].publish(msg)
