#!/bin/zsh

source /root/ros_ws/install/setup.zsh

if [ ! "$(ls -A /root/ros_ws/src/rm_pioneer_config)" ]; then
  git clone https://github.com/chenjunnn/rm_pioneer_config -b ${ROBOT} src/rm_pioneer_config
  colcon build --symlink-install --packages-select rm_pioneer_description rm_pioneer_bringup
fi

ros2 launch rm_pioneer_bringup vision_bringup.launch.py
