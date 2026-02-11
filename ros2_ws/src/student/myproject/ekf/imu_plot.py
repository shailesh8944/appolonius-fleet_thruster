
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np




class YawPlotter(Node):
    def __init__(self):
        super().__init__('yaw_plotter')
        self.imu_yaws = []
        self.ekf_yaws = []
        self.timestamps = []
        self.start_time = None
        self.lock = threading.Lock()

        # Subscribe to topics
        self.imu_sub = self.create_subscription(
            Imu,
            '/sookshma_00/imu/data',
            self.imu_callback,
            10
        )
        self.ekf_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/vessel/pose',
            self.ekf_callback,
            10
        )

    @staticmethod
    def quat_to_eul(quat, order='ZYX', deg=False):
        """Convert quaternion to Euler angles"""
        eul = np.zeros(3, dtype=float)
        
        if order != 'ZYX':
            raise ValueError('Any order other than ZYX is not currently available!')

        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]

        phi = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
        theta = -np.arcsin(2 * (qz * qx - qw * qy))
        psi = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    
        eul = np.array([phi, theta, psi])
        return eul

    def imu_callback(self, msg):
        # Convert ROS quaternion (x,y,z,w) to (w,x,y,z) format
        q = (msg.orientation.w,
             msg.orientation.x, 
             msg.orientation.y, 
             msg.orientation.z)
        
        # Get Euler angles
        angles = self.quat_to_eul(q)
        yaw = angles[2]  # Get yaw (psi)
        yaw_deg = np.rad2deg(yaw)
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = current_time
        
        with self.lock:
            self.imu_yaws.append(yaw_deg)
            self.timestamps.append(current_time - self.start_time)
            
        self.get_logger().debug(f"IMU Yaw: {yaw_deg:.2f}°")

    def ekf_callback(self, msg):
        # Convert ROS quaternion to our format
        # q = (msg.pose.pose.orientation.w,
        #      msg.pose.pose.orientation.x, 
        #      msg.pose.pose.orientation.y, 
        #      msg.pose.pose.orientation.z)
        
        # Get Euler angles
        yaw=msg.pose.pose.orientation.z
        
        yaw_deg = np.rad2deg(yaw)
        
        with self.lock:
            if len(self.timestamps) > 0:
                self.ekf_yaws.append(yaw_deg)
                
        self.get_logger().debug(f"EKF Yaw: {yaw_deg:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = YawPlotter()
    
    # Create figure
    fig, ax = plt.subplots(figsize=(10, 6))
    imu_line, = ax.plot([], [], 'r-', lw=2, label='IMU Yaw')
    ekf_line, = ax.plot([], [], 'b-', lw=2, label='EKF Yaw')
    
    ax.set_xlim(0, 60)
    ax.set_ylim(-180, 180)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Yaw Angle [deg]')
    ax.set_title('Yaw Angle Comparison')
    ax.legend()
    ax.grid(True)

    def update_plot(frame):
        with node.lock:
            # Update both lines
            imu_line.set_data(node.timestamps, node.imu_yaws)
            if len(node.ekf_yaws) == len(node.timestamps):
                ekf_line.set_data(node.timestamps, node.ekf_yaws)
            
            # Auto-adjust x-axis
            if node.timestamps:
                ax.set_xlim(0, max(node.timestamps))
        return imu_line, ekf_line

    ani = animation.FuncAnimation(
        fig, update_plot, frames=300,
        interval=200, repeat=False
    )

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        save_path = './yaw_comparison.gif'
        print("Recording animation...")
        ani.save(save_path, fps=5, writer='pillow')
        print(f"Animation saved to {save_path}")
    except Exception as e:
        print(f"Error saving animation: {e}")
    finally:
        plt.close()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()