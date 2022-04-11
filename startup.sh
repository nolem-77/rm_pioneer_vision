#!/bin/bash

# Uncomment the following line if you want the container to do nothing on startup
# tail -f /dev/null

if [ ! "$(ls -A /root/ros_ws/src/rm_pioneer_config)" ]; then
  git clone https://github.com/chenjunnn/rm_pioneer_config -b ${ROBOT} src/rm_pioneer_config
fi

source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-select rm_pioneer_description rm_pioneer_bringup

source /root/ros_ws/install/setup.bash
ros2 launch rm_pioneer_bringup vision_bringup.launch.py
