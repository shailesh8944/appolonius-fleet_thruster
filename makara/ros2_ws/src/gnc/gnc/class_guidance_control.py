from rclpy.node import Node
from nav_msgs.msg import Odometry
from interfaces.msg import Actuator
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool  # Added
from mav_simulator.module_kinematics import quat_to_eul, llh_to_ned, ssa # Added llh_to_ned
import gnc.module_control as con
import numpy as np
from gnc.class_geofence import GeoFence # Added
from gnc.class_static_obstacle import StaticObstacle # Added

class GuidanceControl(Node):
    def __init__(self, vessel):
        super().__init__('guidance_controller')

        self.vessel = vessel
        # self.odom_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/odometry'
        self.odom_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/odometry_sim'
        self.actuator_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/actuator_cmd'
        self.waypoints_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/waypoints'
        self.geofence_flag_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/geofence_flag' # Added
        self.static_obstacle_flag_topic = f'{self.vessel.vessel_name}_{self.vessel.vessel_id:02d}/static_obstacle_flag' # Added
        
        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
            
        # Create publisher for rudder command
        self.actuator_pub = self.create_publisher(
            Actuator,
            self.actuator_topic,
            10)
        
        # Create publisher for waypoints
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            self.waypoints_topic,
            10)

        # Create publishers for geofence and static obstacle flags (Added)
        self.geofence_flag_pub = self.create_publisher(Bool, self.geofence_flag_topic, 10)
        self.static_obstacle_flag_pub = self.create_publisher(Bool, self.static_obstacle_flag_topic, 10)
        
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second
        
        # Read waypoints
        self.waypoints = np.array(self.vessel.vessel_config['guidance']['waypoints'])
        self.waypoints_type = self.vessel.vessel_config['guidance']['waypoints_type']
        self.waypoint_idx = 1

        self.waypoints_data = {
            'waypoints': self.waypoints,
            'waypoints_type': self.waypoints_type
        }
        
        # Initialize time
        self.start_time = self.get_clock().now()

        # Initialize integrator
        self.ye_int = 0.0
        self.te_int = 0.0
        
        # Select control mode
        self.control_mode = con.pid_control

        # Added for Geofencing and Obstacle Avoidance
        self.llh0 = np.array(self.vessel.vessel_config.get('gps_datum', [0.0, 0.0, 0.0]))
        self.geofences = []
        self.static_obstacles = []

        geofence_data_llh = self.vessel.vessel_config.get('geofence', [])
        if geofence_data_llh and len(geofence_data_llh) > 1:
            for i in range(len(geofence_data_llh) - 1):
                p0_llh = np.array([geofence_data_llh[i][0], geofence_data_llh[i][1], self.llh0[2]])
                p1_llh = np.array([geofence_data_llh[i+1][0], geofence_data_llh[i+1][1], self.llh0[2]])
                p0_ned = llh_to_ned(p0_llh, self.llh0)
                p1_ned = llh_to_ned(p1_llh, self.llh0)
                # GeoFence class expects x0, y0, x1, y1 (2D)
                self.geofences.append(GeoFence(p0_ned[0], p0_ned[1], p1_ned[0], p1_ned[1], id=i))

        static_obstacle_data_llh = self.vessel.vessel_config.get('static_obstacle', [])
        for i, obs_llh_coords in enumerate(static_obstacle_data_llh):
            # Assuming obs_llh_coords is [lat, lon], use llh0 altitude for z
            obs_llh = np.array([obs_llh_coords[0], obs_llh_coords[1], self.llh0[2]])
            if len(obs_llh_coords) == 3: # If [lat, lon, alt] is provided
                obs_llh[2] = obs_llh_coords[2]
            obs_ned = llh_to_ned(obs_llh, self.llh0)
            self.static_obstacles.append(StaticObstacle(obs_ned[0], obs_ned[1], obs_ned[2], id=i))

        self.geo_grad_sum = np.zeros(2)
        self.geo_active_overall = False
        self.static_obs_grad_sum = np.zeros(3) # StaticObstacle provides 3D grad
        self.static_obs_active_overall = False

        self.Kp_avoid_heading = 1 # Used for Geofence and Static Obstacle Avoidance
        
        self.get_logger().info('Guidance Controller initialized')

    def publish_waypoints(self):
        if not self.waypoints_data:
            return
            
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        for waypoint in self.waypoints_data.get('waypoints', []):
            pose = Pose()
            pose.position.x = float(waypoint[0])
            pose.position.y = float(waypoint[1])
            pose.position.z = float(waypoint[2])
            pose_array.poses.append(pose)
        
        self.waypoints_pub.publish(pose_array)
    
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
        current_heading_rad = eul[2] # psi
        
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
            eul[0], # phi
            eul[1], # theta
            eul[2], # psi
            0.0 # rudder angle placeholder, actual rudder state might be elsewhere if needed
        ])

        # --- Geofence and Obstacle Avoidance Logic ---
        agent_x = state[6]
        agent_y = state[7]
        agent_z = state[8]

        # Static Obstacle Logic
        self.static_obs_grad_sum = np.zeros(3)
        self.static_obs_active_overall = False
        if self.static_obstacles: # Check if list is not empty
            for obs in self.static_obstacles:
                active, grad = obs.calc_static_obstacle_grad(agent_x, agent_y, agent_z)
                if active:
                    self.static_obs_active_overall = True
                    self.static_obs_grad_sum += grad
        
        # Geofence Logic
        self.geo_grad_sum = np.zeros(2)
        self.geo_active_overall = False
        if self.geofences: # Check if list is not empty
            for fence in self.geofences:
                active, grad = fence.calc_geofence_grad(agent_x, agent_y) # GeoFence uses 2D
                if active:
                    self.geo_active_overall = True
                    self.geo_grad_sum += grad
        
        # Publish flags
        geo_flag_msg = Bool()
        geo_flag_msg.data = self.geo_active_overall
        self.geofence_flag_pub.publish(geo_flag_msg)
        
        obs_flag_msg = Bool()
        # Geofence has priority: only publish static obstacle active if geofence is NOT active
        obs_flag_msg.data = self.static_obs_active_overall and (not self.geo_active_overall)
        self.static_obstacle_flag_pub.publish(obs_flag_msg)

        # --- Control Decision ---
        rudder_cmd = 0.0 # Initialize rudder_cmd
        ye = 0.0 # Initialize ye for logging if needed, or if control_mode doesn't run

        if self.geo_active_overall:
            self.get_logger().debug('Geofence active, overriding guidance.')
            if np.linalg.norm(self.geo_grad_sum) > 1e-6:
                 psi_des_avoid = np.arctan2(self.geo_grad_sum[1], self.geo_grad_sum[0])
            else: 
                 psi_des_avoid = current_heading_rad # Maintain current heading if gradient is null
            
            heading_error = psi_des_avoid - current_heading_rad
            heading_error = ssa(heading_error)
            rudder_cmd = - self.Kp_avoid_heading * heading_error # minus sign is because of the sign convention of the heading error
            self.ye_int = 0.0 # Reset integrator

        elif self.static_obs_active_overall: # Only if geofence is not active
            self.get_logger().debug('Static obstacle active, overriding guidance.')
            # Use XY components of static_obs_grad_sum for heading
            if np.linalg.norm(self.static_obs_grad_sum[0:2]) > 1e-6:
                psi_des_avoid = np.arctan2(self.static_obs_grad_sum[1], self.static_obs_grad_sum[0])
            else:
                psi_des_avoid = current_heading_rad # Maintain current heading
            
            heading_error = psi_des_avoid - current_heading_rad
            heading_error = ssa(heading_error)
            rudder_cmd = - self.Kp_avoid_heading * heading_error
            self.ye_int = 0.0 # Reset integrator
        
        else:
            # Normal waypoint following
            rudder_cmd_pid, ye_pid, wp_indx = self.control_mode(t, state, self.waypoints, self.waypoint_idx, self.ye_int)
            rudder_cmd = rudder_cmd_pid
            ye = ye_pid # For potential logging or use outside

            if wp_indx != self.waypoint_idx:
                self.get_logger().info(f'Waypoint {self.waypoint_idx} reached')
                self.waypoint_idx = wp_indx
                self.ye_int = 0.0

                if self.waypoint_idx >= len(self.waypoints):
                    self.get_logger().info('***All waypoints reached***')
                    self.waypoint_idx = 1 # Loop back to second waypoint (index 1) as per original logic

            # Calculate time difference for integration (only in normal mode)
            dt = t - self.te_int
            if dt > 0:
                self.ye_int += ye * dt # Use 'ye' from the PID controller
            self.te_int = t # Update te_int regardless of dt to track time for next normal op
        
        # Publish command
        cmd_msg = Actuator()
        cmd_msg.actuator_values = [float(np.rad2deg(rudder_cmd))]
        cmd_msg.actuator_names = ['cs_1']
        cmd_msg.covariance = [0.0]
        self.actuator_pub.publish(cmd_msg)
        
        # Log info
        # self.get_logger().info(
        #     f'\nTime: {t:.2f}s\n'            
        #     f'Rudder Command: {rudder_cmd*180/np.pi:.2f} deg\n'
        # )