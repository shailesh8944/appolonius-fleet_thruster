docker run -it --rm --privileged --name panisim \
    --env="DISPLAY" \
    --env="ROS_DOMAIN_ID=0" \
    -p 8000:8000 \
    -p 9090:9090 \
    --workdir="/workspaces/mavlab" \
    --volume="$(pwd)":"/workspaces/mavlab" \
    --volume="/dev/shm":"/dev/shm" \
    --volume="/dev/sbg":"/dev/sbg" \
    --volume="/dev/uwb":"/dev/uwb" \
    --volume="/dev/ardusimple":"/dev/ardusimple" \
    --volume="/dev/arduino":"/dev/arduino" \
    --volume="/dev/propeller":"/dev/propeller" \
    --volume="/dev/rudder":"/dev/rudder" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/mav_simulator_ros2:1.0 ./mavsim_entry.sh
