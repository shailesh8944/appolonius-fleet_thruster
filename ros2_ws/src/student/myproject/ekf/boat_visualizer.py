# File: boat_visualizer.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math

class BoatVisualizer(Node):
    def __init__(self):
        super().__init__('boat_visualizer')

        self.path_pub = self.create_publisher(Path, '/boat/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/boat/current_pose', 10)

        self.create_subscription(PoseWithCovarianceStamped, '/vessel/pose', self.pose_cb, 10)

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.timer = self.create_timer(0.2, self.publish_path)  # 5Hz

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        # Save latest pose
        self.latest_pose = PoseStamped()
        self.latest_pose.header.stamp = self.get_clock().now().to_msg()
        self.latest_pose.header.frame_id = 'map'
        self.latest_pose.pose = msg.pose.pose

        # Add pose to path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(self.latest_pose)

        # Limit path length
        if len(self.path.poses) > 500:
            self.path.poses.pop(0)

    def publish_path(self):
        if hasattr(self, 'latest_pose'):
            self.pose_pub.publish(self.latest_pose)
            self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = BoatVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
