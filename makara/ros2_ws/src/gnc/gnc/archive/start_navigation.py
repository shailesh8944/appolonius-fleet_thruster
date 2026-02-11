#!/usr/bin/env python3

import numpy as np
import rclpy
import websockets
import asyncio
import yaml
import json
import threading
from class_navigation import Navigation
from class_guidance_control import Guidance_Control
from class_bridge import WebSocketROSBridge_Encoder
from rclpy.executors import MultiThreadedExecutor
from class_vessel import Vessel

def websocket_actuator_thread(gnc):

    async def handler(websocket):
        while True:
            # Only sends the first vessel data through websocket
            # This makes sense as at a time one RasPi will listen to only messages of one vessel
            await websocket.send(gnc.actuator_cmd)
            await asyncio.sleep(1/(gnc.rate))
    
    async def start_server():
        async with websockets.serve(handler, "0.0.0.0", 9001):
            await asyncio.Future()  # Run forever

    def run_event_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(start_server())
        loop.run_forever()

    # Create a new event loop
    new_loop = asyncio.new_event_loop()
    thread = threading.Thread(target=run_event_loop, args=(new_loop,))
    thread.start()    

def main():
    
    inp_fpath = '/workspaces/mavlab/inputs/inputs.yml'

    with open(inp_fpath) as stream:
        inp_data = yaml.safe_load(stream)  # content from inputs.yml stored in data

    rclpy.init()

    ros_nodes = []
    ros_threads = []
    vessels = []
    
    count = 0
    for agent in inp_data['agents']:
        
        if isinstance(agent, str):
            
            with open(agent) as stream:
                agent_data = yaml.safe_load(stream)
            name = agent_data["name"]
            sensors = agent_data["sensors"]

            if "odometry_topic" in agent_data:
                odom_topics = agent_data["odometry_topic"]
            else:
                odom_topics = None

        else:

            name = agent["name"]
            sensors = agent["sensors"]

            if "odometry_topic" in agent:
                odom_topics = agent["odometry_topic"]
            else:
                odom_topics = None


        topic_prefix = f'{name.strip()}_{count:02d}'

        if count == 0:
            topic_prefix_first_agent = topic_prefix
        
        sensor_count = 0
        if sensors is not None:
            for sensor in sensors:
                sensors[sensor_count]['topic'] = sensor["topic"]
                sensor_count += 1

        vessel = Vessel(vessel_data=agent)
        vessels.append(vessel)
        
        ros_nodes.append(Navigation(topic_prefix=topic_prefix, 
            rate=1/inp_data['time_step'],
            sensors=sensors, 
            gps_datum=np.array(inp_data['gps_datum']),
            odom_topics=odom_topics,
            vessel=vessel,
            gravity=inp_data['gravity'],
            density=inp_data['density']))
        
        # ros_nodes.append(Guidance_Control(topic_prefix=topic_prefix, 
        #     rate=1/inp_data['time_step'], 
        #     gps_datum=np.array(inp_data['gps_datum']),            
        #     vessel=vessel,            
        #     odom_topics=odom_topics,            
        #     gravity=inp_data['gravity'],
        #     density=inp_data['density']))
        count += 1

    if inp_data['HIL_flag']:
        bridge = WebSocketROSBridge_Encoder(node= ros_nodes[0].node, websocket_url=f"ws://{inp_data['raspi_ip'].strip()}:9003", topic_prefix=topic_prefix_first_agent)
        websocket_encoder_thread_instance = threading.Thread(target=bridge.run)
        websocket_encoder_thread_instance.start()
    
    websocket_actuator_thread_instance = threading.Thread(target=websocket_actuator_thread, args=(ros_nodes[0],))
    websocket_actuator_thread_instance.start()

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    for gnc_node in ros_nodes:
        executor.add_node(gnc_node.node)
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for gnc_node in ros_nodes:
            gnc_node.node.destroy_node()
        
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()