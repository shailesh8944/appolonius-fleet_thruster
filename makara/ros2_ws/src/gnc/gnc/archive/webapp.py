import rclpy
import numpy as np
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from rclpy.executors import MultiThreadedExecutor
from class_odometry_subscriber import OdometrySubscriber

import sys
import os
module_path = '/workspaces/mavlab/ros2_ws/src/mav_simulator/mav_simulator'
sys.path.append(os.path.abspath(module_path))
import module_kinematics as kin

import threading
import logging
import subprocess
import time
from datetime import datetime
import signal
import yaml

app = Flask(__name__, static_folder='static')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

inp_fpath = '/workspaces/mavlab/inputs/inputs.yml'
with open(inp_fpath) as stream:
    inp_data = yaml.safe_load(stream)  # content from inputs.yml stored in data
llh0 = np.array(inp_data['gps_datum'])
geofence = None
if 'geofence' in inp_data:
    geofence = inp_data['geofence']

ros_thread_instance = None
stop_ros_thread_flag = threading.Event()

odom_topic = None
encoders_topic = None
trajectory = []
encoders = []
TOPIC_TYPES = ['Imu', 'NavSatFix', 'Odometry', 'Actuator', 'Bool']

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Dictionary to keep track of active ROS 2 launch processes
ros2_processes = {}
process_lock = threading.Lock()  # Lock for thread-safe access to ros2_processes

# def launch_ros2_webapp_subscriber(odom_topic, encoders_topic):

#     launch_file = 'webapp_subscriber_launch.py'
#     package = 'gnc'
#     command = ['ros2', 'launch', package, launch_file, f'odom_topic:={odom_topic}', f'encoders_topic:={encoders_topic}']

#     try:
#         # Start the ROS 2 launch process
#         process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
#         logging.info(f'Launched {launch_file} from package {package} with PID {process.pid}')
#         return process
#     except Exception as e:
#         logging.error(f'Failed to launch {launch_file} from package {package}: {str(e)}')
#         return str(e)

def launch_ros2_node(package, launch_file):

    command = ['ros2', 'launch', package, launch_file]

    try:
        # Start the ROS 2 launch process
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        logging.info(f'Launched {launch_file} from package {package} with PID {process.pid}')
        return process
    except Exception as e:
        logging.error(f'Failed to launch {launch_file} from package {package}: {str(e)}')
        return str(e)

def stop_process_with_failsafe(process, timeout=5):
    try:
        # Attempt graceful termination
        # process.terminate()
        # os.kill(process.pid, signal.SIGTERM)
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        
        start_time = time.time()
        while process.poll() is None:
            if time.time() - start_time > timeout:
                process.kill()
                break
            time.sleep(0.1)
        process.wait(timeout=1)
        logging.info(f'Process with PID {process.pid} terminated successfully')
    except Exception as e:
        logging.error(f'Failed to terminate process with PID {process.pid}: {str(e)}')
        return f'Failed to terminate process: {str(e)}'
    return None

def flask_thread():
    # app.run(host='0.0.0.0', port=8500)
    socketio.run(app, host='0.0.0.0', port=8500)

def start_flask():
    # app_thread = threading.Thread(target=flask_thread)
    # app_thread.start()
    flask_thread()

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/map')
def map():
    return render_template('map.html')

@app.route('/odometry_plot')
def odometry_plot():
    return render_template('odometry_plot.html')

@app.route('/mission')
def mission():
    return render_template('mission.html')

@app.route('/odometry')
def odometry():
    return render_template('odometry.html')

@app.route('/get_trajectory', methods=['GET'])
def get_trajectory():
    global node_reference, odom_topic, encoders_topic, stop_ros_thread_flag, ros_thread_instance
    topic = request.args.get('topic')
    if topic == odom_topic or topic == '':
        return jsonify(node_reference.trajectory)
    else:
        odom_topic = topic
        
        stop_ros_thread_flag.set()
        ros_thread_instance.join()

        # Reset stop flag for new thread
        stop_ros_thread_flag = threading.Event()

        # Start a new thread
        ros_thread_instance = threading.Thread(target=ros_thread, args=(odom_topic, encoders_topic,))
        ros_thread_instance.start()
        
        return jsonify(node_reference.trajectory)

@app.route('/get_encoders', methods=['GET'])
def get_encoders():
    global node_reference, odom_topic, encoders_topic, stop_ros_thread_flag, ros_thread_instance
    topic = request.args.get('topic')
    if topic == encoders_topic or topic == '':
        return jsonify(node_reference.encoders)
    else:
        encoders_topic = topic
        
        stop_ros_thread_flag.set()
        ros_thread_instance.join()

        # Reset stop flag for new thread
        stop_ros_thread_flag = threading.Event()

        # Start a new thread
        ros_thread_instance = threading.Thread(target=ros_thread, args=(odom_topic, encoders_topic,))
        ros_thread_instance.start()

        return jsonify(node_reference.encoders)

@app.route('/get_state')
def get_state():
    global node_reference
    dict = {}
    dict['odometry'] = node_reference.trajectory[-1]
    dict['encoders'] = node_reference.encoders[-1]
    return jsonify(dict)

@app.route('/reset', methods=['POST'])
def reset():
    global node_reference
    node_reference.trajectory = []  # Reset the trajectory variable
    node_reference.encoders = []  # Reset the encoders variable
    return 'Trajectory and encoders variables reset'

def has_active_publisher(topic):
    """Check if a given topic has active publishers using 'ros2 topic info'."""
    result = subprocess.run(['ros2', 'topic', 'info', topic], capture_output=True, text=True)
    # Look for "Publisher count: 0" in the output, which indicates no active publishers
    return "Publisher count: 0" not in result.stdout

@app.route('/get_topics', methods=['GET'])
def get_topics():
    try:
        # Get all active topics and their types using ros2 topic list command
        result = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True)        

        if result.returncode != 0:
            return jsonify({"error": "Failed to retrieve topics"}), 500

        topic_type_list = result.stdout.splitlines()
        topic_list = [line.split('[')[0].strip() for line in topic_type_list]
        type_list = [line.split('[')[1][:-1].strip() for line in topic_type_list]
        
        # Filter topics based on types in desired types and that are being published
        filtered_topics = [
            topic for topic, topic_type in zip(topic_list, type_list)
            if has_active_publisher(topic) and topic_type.split('/')[-1] in TOPIC_TYPES
        ]

        return jsonify({"topics": filtered_topics}), 200

    except Exception as e:

        return jsonify({"error": str(e)}), 500

@app.route('/get_odometry_topics', methods=['GET'])
def get_odometry_topics():
    try:
        # Get all active topics and their types using ros2 topic list command
        result = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True)        

        if result.returncode != 0:
            return jsonify({"error": "Failed to retrieve topics"}), 500

        topic_type_list = result.stdout.splitlines()
        topic_list = [line.split('[')[0].strip() for line in topic_type_list]
        type_list = [line.split('[')[1][:-1].strip() for line in topic_type_list]
        
        # Filter topics based on types in desired types and that are being published
        filtered_topics = [
            topic for topic, topic_type in zip(topic_list, type_list)
            if has_active_publisher(topic) and topic_type.split('/')[-1] in ['Odometry']
        ]

        return jsonify({"topics": filtered_topics}), 200

    except Exception as e:

        return jsonify({"error": str(e)}), 500

@app.route('/get_encoder_topics', methods=['GET'])
def get_encoder_topics():
    try:
        # Get all active topics and their types using ros2 topic list command
        result = subprocess.run(['ros2', 'topic', 'list', '-t'], capture_output=True, text=True)

        if result.returncode != 0:
            return jsonify({"error": "Failed to retrieve topics"}), 500

        topic_type_list = result.stdout.splitlines()
        topic_list = [line.split('[')[0].strip() for line in topic_type_list]
        type_list = [line.split('[')[1][:-1].strip() for line in topic_type_list]
        
        # Filter topics based on types in desired types and that are being published
        filtered_topics = [
            topic for topic, topic_type in zip(topic_list, type_list)
            if has_active_publisher(topic) and topic_type.split('/')[-1] in ['Actuator']
        ]

        return jsonify({"topics": filtered_topics}), 200

    except Exception as e:

        return jsonify({"error": str(e)}), 500

@app.route('/start_rosbag', methods=['POST'])
def start_rosbag():

    data = request.json
    selected_topics = data.get('topics', [])
    user_filename = data.get('filename')

    # Construct the file path
    date_str = datetime.now().strftime('%Y_%m_%d')
    base_dir = f"/workspaces/mavlab/bagfiles/{date_str}"

    # Ensure the directory exists
    os.makedirs(base_dir, exist_ok=True)

    # Full path for the bag file
    bag_file_path = os.path.join(base_dir, user_filename)
    
    # Construct the command for rosbag
    command = ["ros2", "bag", "record", "-o", bag_file_path] + selected_topics
    logging.info(command)

    # Start the rosbag record command
    try:
        # Start the ROS 2 launch process
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        logging.info(f'Launched ROS2 bag recording with PID {process.pid}')
        ros2_processes[('ros2-bag', user_filename)] = process
        return jsonify({'message': f'Launched ROS2 bag recording pid: {process.pid}', 'topics': selected_topics}), 200

    except Exception as e:
        
        logging.error(f'Failed to launch ROS2 bag recording: {str(e)}')
        return jsonify({"error": str(e)}), 500

    return

@app.route('/start_ros2', methods=['POST'])
def start_ros2():
    data = request.get_json()
    package_launch_list = data.get('package_launch')

    # if not isinstance(package_launch_list, list):
    #     package_launch_list = [package_launch_list]
    # package_launch_list = request.form.getlist('package_launch')
    
    if not package_launch_list:
        return jsonify({'error': 'No package-launch pairs selected'}), 400

    responses = []
    with process_lock:
        for package_launch in package_launch_list:
            package, launch_file = package_launch.split(':')

            if (package, launch_file) in ros2_processes:
                responses.append({'error': f'{package} - {launch_file} is already running'})
                continue

            process = launch_ros2_node(package, launch_file)
            if isinstance(process, str):
                responses.append({'error': f'Failed to launch {package} - {launch_file}: {process}'})
            else:
                ros2_processes[(package, launch_file)] = process
                responses.append({'message': f'Launched {package} - {launch_file}', 'pid': process.pid})

    return jsonify(responses), 200

@app.route('/stop_ros2', methods=['POST'])
def stop_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    with process_lock:
        process_key = (package, launch_file)
        if process_key not in ros2_processes:
            return jsonify({'error': f'{package} - {launch_file} is not running'}), 400

        process = ros2_processes[process_key]
        error = stop_process_with_failsafe(process)
        if error:
            return jsonify({'error': error}), 500

        del ros2_processes[process_key]
    return jsonify({'message': f'Stopped {package} - {launch_file}'}), 200

@app.route('/active_launches', methods=['GET'])
def active_launches():
    with process_lock:
        active_list = [
            {'package': pkg, 'launch_file': lf, 'pid': proc.pid}
            for (pkg, lf), proc in ros2_processes.items()
        ]
    return jsonify(active_list), 200

@app.route('/reset_ros2', methods=['POST'])
def reset_ros2():
    data = request.get_json()
    package = data.get('package')
    launch_file = data.get('launch_file')

    if not package or not launch_file:
        return jsonify({'error': 'Package and launch file must be specified'}), 400

    process_key = (package, launch_file)
    with process_lock:
        if process_key not in ros2_processes:
            return jsonify({'error': f'{package} - {launch_file} is not running'}), 400

        # Stop the existing process
        process = ros2_processes[process_key]
        error = stop_process_with_failsafe(process)
        if error:
            return jsonify({'error': error}), 500

        # Restart the process
        new_process = launch_ros2_node(package, launch_file)
        if isinstance(new_process, str):
            return jsonify({'error': f'Failed to restart {package} - {launch_file}: {new_process}'}), 500

        # Update the process dictionary
        ros2_processes[process_key] = new_process

    return jsonify({'message': f'Reset {package} - {launch_file} successfully', 'pid': new_process.pid}), 200

@socketio.on('get_state')
def get_state_websocket():

    global node_reference, geofence

    if node_reference.trajectory and node_reference.encoders:
        dict = {}
        dict['odometry'] = node_reference.trajectory[-1]
        dict['encoders'] = node_reference.encoders[-1]
        dict['gps'] = kin.ned_to_llh(node_reference.trajectory[-1][7:10], llh0)[0:2].tolist()
        dict['geofence'] = geofence
        emit('state', dict)
    else:
        logging.info('No data available yet')        
        emit('message', {'data': 'No data available'})

def ros_thread(odom_topic, encoders_topic):

    global stop_ros_thread_flag, node_reference

    rclpy.init()

    ros_nodes = []
    ros_nodes.append(OdometrySubscriber(odom_topic, encoders_topic))

    executor = MultiThreadedExecutor()
    for node in ros_nodes:
        executor.add_node(node.node)
    
    node_reference = ros_nodes[0]

    try:
        while not stop_ros_thread_flag.is_set():
            executor.spin_once()  # Spin once to allow checking the stop flag 
        # executor.spin()

    finally:
        executor.shutdown()
        
        for node in ros_nodes:
            node.node.destroy_node()
        
        rclpy.shutdown()

def main():

    global odom_topic, encoders_topic, ros_thread_instance

    odom_topic = '/kurma_00/odometry_sim'  # Default selected topic
    encoders_topic = '/kurma_00/encoders'  # Default selected topic
    
    ros_thread_instance = threading.Thread(target=ros_thread, args=(odom_topic, encoders_topic,))
    ros_thread_instance.start()
    
    start_flask()

if __name__ == '__main__':
    main()