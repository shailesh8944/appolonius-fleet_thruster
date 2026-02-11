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

def main():
    
    inp_fpath = '/workspaces/mavlab/inputs/inputs.yml'

    with open(inp_fpath) as stream:
        inp_data = yaml.safe_load(stream)  # content from inputs.yml stored in data

    if 'geofence' in inp_data:
        geofence = inp_data['geofence']
    else:
        geofence = None
    
    if 'static_obstacle' in inp_data:
        static_obstacles = inp_data['static_obstacle']
    else:
        static_obstacles = None

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

        sensor_count = 0
        if sensors is not None:
            for sensor in sensors:
                sensors[sensor_count]['topic'] = sensor["topic"]
                sensor_count += 1

        vessel = Vessel(vessel_data=agent)
        vessels.append(vessel)
        
        # ros_nodes.append(Navigation(topic_prefix=topic_prefix, 
        #     rate=1/inp_data['time_step'],
        #     sensors=sensors, 
        #     gps_datum=np.array(inp_data['gps_datum']),
        #     odom_topics=odom_topics,
        #     vessel=vessel,
        #     gravity=inp_data['gravity'],
        #     density=inp_data['density']))
        
        ros_nodes.append(Guidance_Control(topic_prefix=topic_prefix, 
            rate=1/inp_data['time_step'], 
            gps_datum=np.array(inp_data['gps_datum']),
            geofence = geofence,
            static_obstacles=static_obstacles,
            vessel=vessel,            
            odom_topics=odom_topics,            
            gravity=inp_data['gravity'],
            density=inp_data['density']))
        count += 1

    # ROS Multi Thread Execution
    executor = MultiThreadedExecutor()
    for gc_node in ros_nodes:
        executor.add_node(gc_node.node)
    
    try: 
        executor.spin()

    finally:
        executor.shutdown()
        
        for gc_node in ros_nodes:
            gc_node.node.destroy_node()
        
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()