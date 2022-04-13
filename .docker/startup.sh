#!/bin/zsh

# Uncomment the following line if you want the container to do nothing on startup
# tail -f /dev/null

source /root/ros_ws/install/local_setup.zsh
ros2 launch rm_pioneer_bringup vision_bringup.launch.py
