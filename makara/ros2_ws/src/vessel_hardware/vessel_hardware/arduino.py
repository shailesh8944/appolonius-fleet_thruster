#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import Actuator
import numpy as np
import serial
import threading
import time
from mav_simulator.class_world import World

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        
        # Get the namespace from the node
        self.declare_parameter('topic_prefix', '')
        self.topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value

        # Declare parameters for serial port and baud rate
        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Serial connection attributes
        self.serial_conn = None
        self.connected = False
        self.running = False
        self.current_command = "0,0"
        self.command_thread = None
        
        # Subscribe to the Actuator topic
        self.subscription = self.create_subscription(
            Actuator,
            f'{self.topic_prefix}/actuator_cmd',
            self.actuator_callback,
            10)
        
        # Publisher for the thrust command topic (for logging/monitoring)
        self.publisher = self.create_publisher(String, f'{self.topic_prefix}/thrust_monitor', 10)
        
        # Setup serial connection
        self.connect_to_arduino()
        
        # Start the command sender thread if connected
        if self.connected:
            self.start_command_thread()
        
        # Create a timer for checking serial connection status
        self.timer = self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info(f"Arduino Serial Node initialized. Port: {self.serial_port}")

    def connect_to_arduino(self):
        """Connect to the Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            # Allow time for the Arduino to reset after connection
            time.sleep(2)
            self.connected = True
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Error connecting to Arduino: {e}")
            self.connected = False
            return False

    def check_connection(self):
        """Periodically check if the connection is still active"""
        if not self.connected or (self.serial_conn and not self.serial_conn.is_open):
            self.get_logger().warn("Serial connection lost. Attempting to reconnect...")
            self.connected = False
            
            # Stop command thread if running
            self.stop_command_thread()
            
            # Try to close the connection if it exists
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
            
            # Try to reconnect
            if self.connect_to_arduino():
                self.start_command_thread()

    def actuator_callback(self, msg):
        """Process incoming actuator commands and update current command"""
        # Extract the propeller and rudder values from the Actuator message
        # propeller = np.clip(msg.actuator_values[1]/800, -1, 1)
        propeller = 0.5
        rudder = np.clip(float(msg.actuator_values[0])/35, -1, 1)
        
        # Format the command for the Arduino
        # The Arduino code expects: steering,throttle
        # So we map rudder to steering and propeller to throttle
        self.current_command = f"{rudder:.2f},{propeller:.2f}"
        
        # Log the command to ROS topic
        thrust_command_msg = String()
        thrust_command_msg.data = f"{rudder:.2f},{propeller:.2f}"
        self.publisher.publish(thrust_command_msg)
        # self.get_logger().info(f"Command received: {thrust_command_msg.data}")

    def send_command(self, command):
        """Send command to Arduino via serial"""
        if not self.connected:
            return False
        
        try:
            # Add newline as per Arduino code's expectation
            full_command = command + "\n"
            self.serial_conn.write(full_command.encode())
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")
            self.connected = False
            return False

    def read_response(self):
        """Read response from Arduino if available"""
        if not self.connected:
            return None
        
        try:
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('utf-8').strip()
                self.get_logger().info(f"Arduino response: {response}")
                return response
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading response: {e}")
            self.connected = False
            return None

    def command_sender_loop(self):
        """Thread function to continuously send the current command"""
        while self.running and self.connected:
            try:
                # Send the current command to Arduino
                self.send_command(self.current_command)
                
                # Small delay to avoid flooding the serial port
                time.sleep(0.05)
                
                # Check for any responses
                self.read_response()
                
            except Exception as e:
                self.get_logger().error(f"Error in command sender: {e}")
                self.connected = False

    def start_command_thread(self):
        """Start the thread to continuously send commands"""
        self.running = True
        self.command_thread = threading.Thread(target=self.command_sender_loop)
        self.command_thread.daemon = True
        self.command_thread.start()
        self.get_logger().info("Command sender thread started")
    
    def stop_command_thread(self):
        """Stop the command sender thread"""
        self.running = False
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=1)
            self.get_logger().info("Command sender thread stopped")

    def on_shutdown(self):
        """Cleanup when the node is shutting down"""
        # Stop motors
        if self.connected:
            self.send_command("0,0")
            time.sleep(0.1)
        
        # Stop the command thread
        self.stop_command_thread()
        
        # Close the serial connection
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")


def main(args=None):

    rclpy.init(args=args)
    
    arduino_node = ArduinoSerialNode()
    
    try:
        rclpy.spin(arduino_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Perform cleanup
        arduino_node.on_shutdown()
        
        # Destroy the node explicitly
        arduino_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
