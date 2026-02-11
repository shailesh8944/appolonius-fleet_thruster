cd /workspaces/mavlab/ros2_ws
colcon build
source ~/.bashrc && source /opt/ros/humble/setup.bash && source /workspaces/mavlab/ros2_ws/install/setup.bash
# ros2 run gnc gc
ros2 launch gnc gc_launch.py