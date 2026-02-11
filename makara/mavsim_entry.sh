cd ros2_ws
colcon build
source /opt/ros/humble/setup.bash && source /workspaces/mavlab/ros2_ws/install/setup.bash
# ros2 run mav_simulator simulate
ros2 launch mav_simulator mav_simulator_launch.py 