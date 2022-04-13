export TERM=xterm-256color
source /root/ros_ws/install/setup.zsh
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"

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
