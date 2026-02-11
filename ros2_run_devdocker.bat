@echo off
setlocal enabledelayedexpansion

:: Check if an argument is supplied
if "%~1"=="" (
    echo No arguments supplied! bash assumed
    set cmd=bash
) else (
    set cmd=%1
)

:: Run the Docker container
docker run -it --rm --privileged --name oe5005 ^
    --net=host ^
    --env="DISPLAY" ^
    --env="ROS_DOMAIN_ID=42" ^
    --workdir="/workspaces/mavlab" ^
    --volume="%cd%":"/workspaces/mavlab" ^
    --volume="/dev/shm":"/dev/shm" ^
    --volume="/dev/sbg":"/dev/sbg" ^
    --volume="/dev/ardusimple":"/dev/ardusimple" ^
    --volume="/dev/propeller":"/dev/propeller" ^
    --volume="/dev/rudder":"/dev/rudder" ^
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ^
    abhilashiit/oe5005:1.0 !cmd!
