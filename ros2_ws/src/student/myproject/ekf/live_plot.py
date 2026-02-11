# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import threading
# import numpy as np
# class LivePlotter(Node):
#     def __init__(self):
#         super().__init__('live_plotter')

#         self.xs = []
#         self.ys = []

#         self.subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/vessel/pose',  # your EKF output
#             self.pose_callback,
#             10
#         )

#     def pose_callback(self, msg):
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         self.xs.append(x)
#         self.ys.append(y)
#         self.get_logger().info(f"Received: x={x:.2f}, y={y:.2f}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = LivePlotter()
#     ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     ros_spin_thread.start()

#     fig, ax = plt.subplots()
#     line, = ax.plot([], [], 'b-', lw=2)
#     ax.set_xlim(-50, 50)  # Set according to your environment
#     ax.set_ylim(-50, 50)
#     ax.set_xlabel('X [m]')
#     ax.set_ylabel('Y [m]')
#     ax.set_title('Boat Live Path')

#     def update(frame):
#         if node.xs:
#             line.set_data(node.xs, node.ys)
#         return line,

#     ani = animation.FuncAnimation(fig, update, interval=200)

#     try:
#         plt.show()
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3

#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        # EKF estimated positions
        self.x = 0.0
        self.y = 0.0
        self.ekf_xs = []
        self.ekf_ys = []
        # UWB ground truth positions
        self.uwb_xs = []
        self.uwb_ys = []
        self.lock = threading.Lock()

        # Subscribe to both topics
        self.ekf_sub = self.create_subscription(
            Odometry,
            '/er/odometry',
            self.ekf_callback,
            10
        )
        self.uwb_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/sookshma_00/uwb',
            self.uwb_callback,
            10
        )

    def ekf_callback(self, msg:Odometry):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        with self.lock:
            self.ekf_xs.append(self.x)
            self.ekf_ys.append(self.y)
        self.get_logger().info(f"EKF: x={self.x:.2f}, y={self.y:.2f}")

    def uwb_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self.lock:
            self.uwb_xs.append(x)
            self.uwb_ys.append(y)
        self.get_logger().info(f"UWB: x={x:.2f}, y={y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()
    
    # Set up the plot
    fig, ax = plt.subplots()
    ekf_line, = ax.plot([], [], 'b-', lw=2, label='EKF Estimate')
    uwb_line, = ax.plot([], [], 'r-', lw=2, label='UWB Ground Truth')
    
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Vessel Trajectory Comparison')
    ax.legend()
    ax.grid(True)

    frames_to_capture = 300  # Adjust this number based on your bag file duration
    
    def update_plot(frame):
        with node.lock:
            ekf_line.set_data(node.ekf_xs, node.ekf_ys)
            uwb_line.set_data(node.uwb_xs, node.uwb_ys)
        return ekf_line, uwb_line

    # Create animation with fixed number of frames
    ani = animation.FuncAnimation(fig, update_plot, frames=frames_to_capture, 
                                interval=200, repeat=False)

    # Start ROS2 in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        plt.show()
        save_path = './vessel_trajectory_comparison2.gif'
        print("Recording animation... Please wait.")
        # Save animation with progress callback
        ani.save(save_path, fps=5, writer='ffmpeg',
                progress_callback=lambda i, n: print(f"Saving frame {i} of {n}"))
        print("Successfully saved animation to /tmp/vessel_trajectory_comparison.gif")
    except Exception as e:
        print(f"Failed to save animation: {e}")
    finally:
        # Clean up
        plt.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Node shut down successfully")

if __name__ == '__main__':
    main()