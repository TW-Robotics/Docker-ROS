# Instructions for Docker 

The following readme gives a short introduction on how to use the docker container on Linux and on Windows.   
Note that it is not possible to communicate with other hosts (e.g. Turtlebot) via wifi nor lan if you use docker on Windows.

## Prequisites

1. Install docker (here are the instructions: [link](https://docs.docker.com/install/linux/docker-ce/ubuntu/) )
2. Copy the downloaded files to a destination of your choice. e.g. ~/Documents/Docker/FHTW/

### Windows only

3. Install VcXsrv as X11-Server: [link](https://sourceforge.net/projects/vcxsrv/files/latest/download)   
Make sure that you allow VcXsrv access to public and private networks


## Linux

1. Now you can either build the container from source or pull it from the docker hub
    1. Build locally:
    ```
    docker build -t "fhtw/ros-melodic" --rm .
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

## Windows

1. Start VcXsrv (XLaunch) with following configuration:   
![VcXsrv Configuration](./XmingConfig.PNG)
2. Now you can either build the container from source or pull it from the docker hub
    1. Build:
        1. Navigate to the downloaded folder using file explorer
        2. Double klick on [build_docker_container.bat](./build_docker_container.bat)
    2. Pull:
        ```
        docker pull georgno/fhtw-ros:latest
        ```
3. To start the docker container double klick on either [run_docker_from_hub.bat](./run_docker_from_hub.bat) or [run_docker_from_local_build.bat](./run_docker_from_local_build.bat)
On the first start docker will ask for permissions to mount catkin_ws/src folder (for more see below).

## Development inside the Docker Container

To make it easier to develope within the docker container, create a folder "./catkin_ws/src/" (which must be located directly in the folder from which you run the run_docker_from_hub/local). This folder is mounted into the docker container to "/home/fhtw_user/catkin_ws/src/fhtw". This allows you to save your projects.   

If you want to work with a IDE we recommand to use Visual Studio Code and the following plugins:

- Remote Development (ms-vscode-remote.vscode-remote-extensionpack) 
- ROs (ms-iot.vscode-ros)
- C++ Intellisense (austin.code-gnu-global)

Further to work with multiple terminals inside docker we recommend using [tmux](https://thoughtbot.com/blog/a-tmux-crash-course)
