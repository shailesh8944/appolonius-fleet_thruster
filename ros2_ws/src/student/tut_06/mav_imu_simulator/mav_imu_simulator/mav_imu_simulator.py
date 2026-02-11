import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

import sys
sys.path.append('/workspaces/mavlab/')

from ros2_ws.src.student.tut_03.read_input import read_input
from ros2_ws.src.student.tut_03.class_vessel import Vessel
from ros2_ws.src.student.tut_03.module_kinematics import eul_to_quat, eul_to_rotm, quat_to_eul, rotm_to_eul

class MavImuSimulator(Node):
    def __init__(self):
        super().__init__('mav_imu_simulator')

        # Read vessel parameters and create vessel instance
        vessel_params, hydrodynamic_data = read_input()
        self.vessel = Vessel(vessel_params, hydrodynamic_data)
        
        self.g = vessel_params['g']
        
        # Find IMU sensors and create publishers
        self.imu_sensors = []
        self.imu_publishers = []
        self.imu_timers = []

        self.vessel_state = np.zeros(13)
        
        count = 0
        for sensor in vessel_params['sensors']:
            if sensor['sensor_type'] == 'IMU':                
                self.imu_sensors.append({
                    'id': count,
                    'position': np.array(sensor['sensor_location']),
                    'orientation': np.array(sensor['sensor_orientation']),
                    'topic': sensor['topic'],
                    'rate': sensor['rate'],
                    'orientation_covariance': np.array(sensor['noise'][0]['orientation_covariance']),
                    'angular_velocity_covariance': np.array(sensor['noise'][2]['angular_velocity_covariance']),
                    'linear_acceleration_covariance': np.array(sensor['noise'][1]['linear_acceleration_covariance'])
                })
                
                # Create publisher for this IMU
                self.imu_publishers.append(
                    self.create_publisher(Imu, self.imu_sensors[count]['topic'], 10)
                )
                
                # Create timer for this IMU
                self.imu_timers.append(
                    self.create_timer(
                        1/self.imu_sensors[count]['rate'],
                        lambda i=count: self.publish_imu_messages(i)
                    )
                )
                count += 1
        
        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/mav_odom',
            self.odom_callback,
            10
        )

    def compute_imu_orientation(self, 
            vessel_state, 
            sensor_position, 
            sensor_orientation, 
            eul_flag=False
        ):
        """
        Compute IMU orientation quaternion

        Args:
            vessel_state: 13x1 vessel state vector
            sensor_position: 3x1 sensor position vector
            sensor_orientation: 3x1 sensor orientation vector
            eul_flag: boolean flag to indicate if the orientation is in Euler angles

        Returns:
            quat_eul: 4x1 quaternion vector if eul_flag is False, otherwise 3x1 Euler angles
        """

        # Extract BCS orientation (assuming ZYX Euler angles) wrt GCS frame
        Theta_gb = vessel_state[9:12]        
        
        # Sensor frame position and orientation with respect to BCS (assuming ZYX Euler angles)
        r_bs = sensor_position
        Theta_bs = sensor_orientation

        # Sensor frame orientation wrt GCS frame (assuming ZYX Euler angles) - Reported by sensor
        sensor_reported_orientation = np.zeros(3)

        #===========================================================================
        # TODO: Calculate orientation (in Euler angles) reported by sensor 
        # in sensor frame
        #===========================================================================

        # Write your code here
        pass

        #===========================================================================

        if not eul_flag:
            # Convert orientation to quaternion
            quat_eul = eul_to_quat(sensor_reported_orientation)
        else:
            quat_eul = sensor_reported_orientation
        
        return quat_eul

    def compute_imu_angular_velocity(self, 
            vessel_state, 
            sensor_position, 
            sensor_orientation
        ):
        """
        Compute IMU angular velocity

        Args:
            vessel_state: 13x1 vessel state vector
            sensor_position: 3x1 sensor position vector
            sensor_orientation: 3x1 sensor orientation vector

        Returns:
            omega_s: 3x1 angular velocity vector in sensor frame
        """
        # Extract vessel angular velocity in body frame
        omega_b = vessel_state[3:6]

        # Sensor position and orientation with respect to BCS
        r_bs = sensor_position
        Theta_bs = sensor_orientation

        # Compute angular velocity in sensor frame
        omega_s = np.zeros(3)

        #===========================================================================
        # TODO: Calculate angular velocity in sensor frame
        #===========================================================================

        # Write your code here
        pass

        #===========================================================================
        return omega_s

    def compute_imu_linear_acceleration(self, 
            vessel_state, 
            sensor_position, 
            sensor_orientation
        ):
        """
        Compute IMU linear acceleration including centripetal effects

        Args:
            vessel_state: 13x1 vessel state vector
            sensor_position: 3x1 sensor position vector
            sensor_orientation: 3x1 sensor orientation vector

        Returns:
            a_s: 3x1 linear acceleration vector in sensor frame
        """
        # Extract vessel linear and angular velocities
        v = vessel_state[0:3]
        omega = vessel_state[3:6]
        eul = vessel_state[9:12]

        # Sensor position and orientation with respect to BCS
        r_bs = sensor_position
        Theta_bs = sensor_orientation

        imu_eul = self.compute_imu_orientation(vessel_state, sensor_position, sensor_orientation, eul_flag=True)

        a_s = np.zeros(3)

        #===========================================================================
        # TODO: Calculate linear acceleration in sensor frame
        #===========================================================================

        # Write your code here
        pass

        #===========================================================================

        # Add gravity component to the linear acceleration
        gravity = np.array([0, 0, -self.g])
        gravity_s = eul_to_rotm(imu_eul) @ gravity

        a_s = a_s + gravity_s
        
        return a_s

    def odom_callback(self, msg):
        """Handle incoming odometry messages"""
        # Extract state from odometry message
        self.vessel_state = np.zeros(13)
        self.vessel_state[0:3] = [msg.twist.twist.linear.x, 
                            msg.twist.twist.linear.y, 
                            msg.twist.twist.linear.z]
        self.vessel_state[3:6] = [msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z]
        self.vessel_state[6:9] = [msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z]
        # Extract Euler angles from quaternion
        self.vessel_state[9:12] = quat_to_eul([msg.pose.pose.orientation.w,
                                          msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z])
        
    def publish_imu_messages(self, i):
        """Publish IMU messages"""

        sensor = self.imu_sensors[i]

        # Update each IMU        
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = f'imu_{i:02d}'
        
        # Compute IMU measurements
        orientation = self.compute_imu_orientation(
            self.vessel_state, sensor['position'], sensor['orientation'])
        angular_velocity = self.compute_imu_angular_velocity(
            self.vessel_state, sensor['position'], sensor['orientation'])
        linear_acceleration = self.compute_imu_linear_acceleration(
            self.vessel_state, sensor['position'], sensor['orientation'])
        
        # Populate IMU message
        imu_msg.orientation.w = orientation[0]
        imu_msg.orientation.x = orientation[1]
        imu_msg.orientation.y = orientation[2]
        imu_msg.orientation.z = orientation[3]
        
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        
        # Set covariances from input file
        imu_msg.orientation_covariance = sensor['orientation_covariance'].flatten()
        imu_msg.angular_velocity_covariance = sensor['angular_velocity_covariance'].flatten()
        imu_msg.linear_acceleration_covariance = sensor['linear_acceleration_covariance'].flatten()
        
        # Publish IMU message
        self.imu_publishers[i].publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MavImuSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
