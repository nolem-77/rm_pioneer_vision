#!/bin/zsh

source /root/ros_ws/install/setup.zsh
source /opt/intel/openvino_2021/bin/setupvars.sh

ros2 launch rm_pioneer_vision composable_bringup.launch.py
