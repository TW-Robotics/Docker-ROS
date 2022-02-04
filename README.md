# Content
This repo contains a [Dockerfile](Dockerfile) and runfiles for gpu accelerated OpenGL rendering.   
$\rightarrow$ Offloads OpenGL rendering to GPU 

## Instructions for Docker 

The following readme gives a short introduction on how to use the gpu accelerated docker container on Linux.  
Note this should also work with Windows 11 and WSL2 with build higher build 22000.* and an NVIDIA GPU [link](https://github.com/microsoft/wslg)
### Prequisites

0. Install the nvidia driver for your gpu: [link](https://www.nvidia.de/Download/index.aspx?lang=en)
1. Install docker (here are the instructions: [link](https://docs.docker.com/install/) )
2. Install nvidia-docker2 (here are the instructions: [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) ) [not needed for Windows 11]
3. Copy the downloaded files to a destination of your choice. e.g. ~/Documents/Docker/FHTW/


### Linux

1. Now you can either build the container from source or pull it from the docker hub
    1. Build locally:
    ```
    docker build -t "fhtw/ros-melodic-gpu" --rm .
    ```
    2. Use the image on the docker hub:
    ```
    docker pull georgno/fhtw-ros:latest
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

To make it easier to develope within the docker container, create a folder "./catkin_ws/src/" (which must be located directly in the folder from which you run the run_docker_from_hub/local). This folder is mounted into the docker container to "/home/fhtw_user/catkin_ws/src/fhtw". This allows you to save your projects.   

If you want to work with a IDE we recommand to use Visual Studio Code and the following plugins:

- Remote Development (ms-vscode-remote.vscode-remote-extensionpack) 
- ROS (ms-iot.vscode-ros)
- C++ Intellisense (austin.code-gnu-global)

Further to work with multiple terminals inside docker we recommend using [tmux](https://thoughtbot.com/blog/a-tmux-crash-course)
