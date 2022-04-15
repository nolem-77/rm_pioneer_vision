#!/bin/zsh

# Clone config repo if it doesn't exist
if [ ! "$(ls -A /root/ros_ws/src/rm_pioneer_config)" ]; then
    git clone https://github.com/chenjunnn/rm_pioneer_config -b ${ROBOT} src/rm_pioneer_config
else
    export ROBOT=$(git -C src/rm_pioneer_config branch --show-current)
fi

case $ROBOT in
    hero)
        export ROS_DOMAIN_ID=1
        ;;
    standard3)
        export ROS_DOMAIN_ID=3
        ;;
    standard4)
        export ROS_DOMAIN_ID=4
        ;;
    standard5)
        export ROS_DOMAIN_ID=5
        ;;
    guard)
        export ROS_DOMAIN_ID=6
        ;;
    *)
        echo "Unknown robot type: $ROBOT"
        ;;
esac

source /opt/ros/galactic/setup.zsh
colcon build --symlink-install --packages-select rm_pioneer_description rm_pioneer_bringup

exec "$@"
