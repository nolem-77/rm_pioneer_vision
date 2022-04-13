# rm_pioneer_vision

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/chenjunnn/rm_pioneer_vision/actions/workflows/ci.yml/badge.svg)](https://github.com/chenjunnn/rm_pioneer_vision/actions/workflows/ci.yml)

## 使用 Docker 部署 (Recommended)

1. [Install Docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)

    ```bash
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

    ```bash
    sudo usermod -aG docker $USER
    newgrp docker 
    ```

3. [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot)

    ```bash
    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service
    ```

5. [Authenticating to the Container registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry)

    ```bash
    export CR_PAT=YOUR_TOKEN
    echo $CR_PAT | docker login ghcr.io -u USERNAME --password-stdin
    ```

3. 拉取镜像

    ```bash
    docker pull ghcr.io/chenjunnn/rm_pioneer_vision:latest
    ```

4. 创建容器并运行
  
    ```bash
    docker run --name vision --restart always --privileged \
    -v /dev/bus/usb:/dev/bus/usb/ --network host \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/config/:/root/ros_ws/src/rm_pioneer_config:rw \
    -e ROBOT=standard3 -it ghcr.io/chenjunnn/rm_pioneer_vision:latest
    ```
    > Tips: 需要在图形化界面中启动容器，否则在容器内无法正常使用 GUI

5. 使用`exec`命令进入容器并启动 rviz2

    ```bash
    # opening up xhost to local connection
    xhost +local:
    # run zsh in vision container
    docker exec -it vision zsh
    # run rviz2 in vision container
    rviz2 -d vision.rviz
    ```
    
