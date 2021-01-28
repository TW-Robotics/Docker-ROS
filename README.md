# Instructions for Docker 

The following readme gives a short introduction on how to use the docker container on Linux with gpu support.   

## Prequisites

1. Install docker (here are the instructions: [Ubuntu](https://docs.docker.com/install/linux/docker-ce/ubuntu/) )
2. Install CUDA on your PC  (here are the instructions: [link](https://developer.nvidia.com/cuda-downloads) )
3. Install nvidia-docker2 on your PC (here are the instructions: [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) )
4. Copy the downloaded files to a destination of your choice. e.g. ~/Documents/Docker/FHTW/


## Linux

1. Now you can either build the container from source or pull it from the docker hub (recommended).
    1. Build locally:
    ```
    docker build -t "fhtw/ros:latest-gpu" --rm .
    ```
    2. Use the image on the docker hub:
    ```
    docker pull georgno/fhtw-ros:master-gpu
    ```
2. To start the docker container execute the following commands:
   1. With local build:
   ```
   bash run_docker_from_local_build.sh
   ```
   2. With hub:
   ```
   bash run_docker_from_hub.sh
   ```


## Development inside the Docker Container

To make it easier to develope within the docker container, create a folder "./catkin_ws/src/" (which must be located directly in the folder from which you run the run_docker_from_hub/local). This folder is mounted into the docker container to "/home/fhtw_user/catkin_ws/src/fhtw". This allows you to save your projects on your host computer and execute them in the docker container.   

If you want to work with a IDE we recommand to use Visual Studio Code and the following plugins:

- Remote Development (ms-vscode-remote.vscode-remote-extensionpack) 
- ROS (ms-iot.vscode-ros)
- C++ Intellisense (austin.code-gnu-global)

Further to work with multiple terminals inside docker we recommend using [tmux](https://thoughtbot.com/blog/a-tmux-crash-course) which is set as the default shell.


## Commit changes to the docker container
To store changes (such as newly installed software) you need to commit these changes from your command line (we recommand powershell for windows and the normal terminal for linux).
```
# docker commit [CONTAINER_ID] fhtw_ros_local:latest -m "Commit message"
```
