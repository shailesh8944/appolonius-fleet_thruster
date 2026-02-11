#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import Actuator
import sys, termios, tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('asv_teleop_node')
        
        self.publisher = self.create_publisher(Actuator, '/sookshma_00/actuator_cmd', 10)
        self.propeller = 0.0  # Throttle
        self.rudder = 0.0  # Steering
        
        self.get_logger().info("ASV Teleop Node Initialized. Use W/S for throttle, A/D for steering. Q to quit.")
        self.run_teleop()
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key
    
    def run_teleop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w':
                    self.propeller += 50.0
                    if self.propeller > 800.0:
                        self.propeller = 0.0  # Reset to loop back
                elif key == 's':
                    self.propeller -= 50.0
                    if self.propeller < -800.0:
                        self.propeller = -800.0  # Clamp min
                elif key == 'a':
                    self.rudder = max(self.rudder - 5, -35)
                elif key == 'd':
                    self.rudder = min(self.rudder + 5, 35)
                elif key == 'q':
                    self.get_logger().info("Exiting teleop...")
                    break
                else:
                    continue
                
                # **LOG PROPULSION VALUE BEFORE PUBLISHING**
                self.get_logger().info(f"Throttle Before Publishing: {self.propeller}")

                msg = Actuator()
                # Set actuator values as float arrays
                msg.actuator_values = [float(self.rudder)]
                msg.actuator_names = ["rudder"]
                msg.covariance = [0.0]

                self.publisher.publish(msg)
                self.get_logger().info(f"Sent Command - Throttle: {self.propeller}, Steering: {self.rudder}")
        
        except Exception as e:
            self.get_logger().error(f"Error in teleop: {e}")

        finally:
            msg = Actuator()
            msg.actuator_values = [0.0]
            msg.actuator_names = ["rudder"]
            msg.covariance = [0.0]
            self.publisher.publish(msg)
            self.get_logger().info("Stopping ASV...")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

