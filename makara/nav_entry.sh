cd /workspaces/mavlab/ros2_ws
rm -rf build/gnc install/gnc log/gnc
colcon build
source ~/.bashrc && source /opt/ros/humble/setup.bash && source /workspaces/mavlab/ros2_ws/install/setup.bash
# ros2 run gnc nav
ros2 launch gnc nav_launch.py