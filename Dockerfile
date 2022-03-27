FROM osrf/ros:galactic-desktop

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t jispwoso \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting \
    && rm -rf /var/lib/apt/lists/*

# install clangd and autopep8
RUN apt-get update && \
    apt-get install -y clangd-12 python3-autopep8 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 100 && \
    rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws/

# copy source code
COPY . src/rm_pioneer_vision

# install dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# build source
SHELL [ "/bin/zsh", "-c" ]
RUN source /opt/ros/galactic/setup.zsh && \
    colcon build \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --symlink-install

RUN echo \
    $'export TERM=xterm-256color \n\
    source /root/ros_ws/install/setup.zsh \n\
    eval "$(register-python-argcomplete3 ros2)" \n\
    eval "$(register-python-argcomplete3 colcon)"' >> ~/.zshrc

ENV ROBOT=standard3

CMD [ "src/startup.sh" ]