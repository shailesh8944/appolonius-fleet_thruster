cd /workspaces/mavlab/ros2_ws
colcon build
source ~/.bashrc && source /opt/ros/humble/setup.bash && source /workspaces/mavlab/ros2_ws/install/setup.bash
# ros2 launch sbg_driver sbg_device_launch.py &
# ros2 launch ublox_gps ublox_gps_node-launch.py &
# ros2 run gnc gc
ros2 launch gnc gnc_launch.py