import numpy as np
import websocket
import json
import asyncio
import time
from std_msgs.msg import String
from interfaces.msg import Actuator

class WebSocketROSBridge_Encoder:
    def __init__(self, node, websocket_url, topic_prefix=''):
        # Initialize ROS node
        self.node = node
        self.websocket_url = websocket_url
        self.topic_prefix = topic_prefix
        
        self.ws = None
        self.rudder_encoder = 0.0
        self.propeller_encoder = 0.0

        self.pub = self.node.create_publisher(Actuator, f'/{self.topic_prefix}/encoders', 90)
            
    def on_message(self, ws, message):
        msg = json.loads(message)
        self.rudder_encoder = np.float64(msg['rudder'])
        self.propeller_encoder = np.float64(msg['propeller'])
        
        current_time = self.node.get_clock().now()

        # Create actuator message
        act = Actuator()
        act.header.stamp = current_time.to_msg()
        act.rudder = self.rudder_encoder
        act.propeller = self.propeller_encoder

        encoders_rms = np.array([1e-1 * np.pi / 180, 1e-1 / 60])
        encoders_cov = np.diag(encoders_rms ** 2)
        
        act.covariance = encoders_cov.flatten()

        self.pub.publish(act)

    def on_error(self, ws, error):
        self.node.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.node.get_logger().info(f"WebSocket closed with status code {close_status_code}: {close_msg}")

    def on_open(self, ws):
        self.node.get_logger().info("WebSocket connected")

    def run(self):        
        while True:
            time.sleep(5)
            try:
                # Connect to the WebSocket server
                # websocket.enableTrace(True)  # Enable trace for debugging (optional)
                ws = websocket.WebSocketApp(self.websocket_url,
                                        on_message=self.on_message,
                                        on_error=self.on_error,
                                        on_close=self.on_close)
                ws.on_open = self.on_open
                ws.run_forever()
            except:
                pass

if __name__ == '__main__':

    # Replace 'ws://example.com/socket' with your WebSocket URL
    # Replace '/ros_topic' with the name of your ROS topic
    bridge = WebSocketROSBridge_Encoder(websocket_url='ws://localhost:9002',
                                topic_prefix='makara_00')
    bridge.run()

    
    
