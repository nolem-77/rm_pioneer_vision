FROM osrf/ros:galactic-desktop

# install autopep8 & wget
RUN apt-get update && \
    apt-get install -y python3-autopep8 wget && \
    rm -rf /var/lib/apt/lists/*

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t jispwoso \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting \
    && rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws/

# copy source code
COPY . src/rm_pioneer_vision

# install dependencies
RUN apt-get update && \
    apt-get install -y ros-galactic-xacro && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# build source
RUN . /opt/ros/galactic/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --symlink-install

RUN echo \
    'export TERM=xterm-256color \n\
    source /root/ros_ws/install/setup.zsh \n\
    eval "$(register-python-argcomplete3 ros2)" \n\
    eval "$(register-python-argcomplete3 colcon)"' >> ~/.zshrc

ENV ROBOT=standard3

CMD [ "src/rm_pioneer_vision/startup.sh" ]