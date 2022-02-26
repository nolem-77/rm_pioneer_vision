# rm_pioneer_vision

![Build Status](https://github.com/chenjunnn/rm_pioneer_vision/actions/workflows/ci.yml/badge.svg)

## 使用 Docker 部署 (Recommended)

1. [Install Docker using the convenience script](https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script)

    ```bash
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    ```

2. [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) (Optional)

    ```bash
    sudo usermod -aG docker $USER
    newgrp docker 
    ```

3. [Authenticating to the Container registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry#authenticating-to-the-container-registry)

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
    docker run --name=vision --network=host --privileged -d \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    -e ROBOT=guard \
    ghcr.io/chenjunnn/rm_pioneer_vision:latest
    ```

5. 使用`exec`命令进入容器

    ```bash
    docker exec -it vision zsh
    ```

6. 设置开机自启

    ```bash
    sudo systemctl enable docker.service
    sudo systemctl enable containerd.service
    docker update --restart always vision
    ```
