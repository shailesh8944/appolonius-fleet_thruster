cd /workspaces/mavlab/ros2_ws
colcon build
source /opt/ros/humble/setup.bash && source /workspaces/mavlab/ros2_ws/install/setup.bash
python3 /workspaces/mavlab/ros2_ws/src/gnc/gnc/webapp.py