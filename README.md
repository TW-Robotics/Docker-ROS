# Working with the Docker Container

- [Working with the Docker Container](#working-with-the-docker-container)
  - [About Containers](#about-containers)
  - [The Docker Container](#the-docker-container)
    - [Starting the Docker Container](#starting-the-docker-container)
      - [Devcontainer (recommended)](#devcontainer-recommended)
      - [Convenience Script (only recommended if you know your way around the terminal)](#convenience-script-only-recommended-if-you-know-your-way-around-the-terminal)
        - [Windows](#windows)
        - [Linux](#linux)
      - [Switch between terminal/desktop, nvidia/amd/standard environment](#switch-between-terminaldesktop-nvidiaamdstandard-environment)
        - [Devcontainer](#devcontainer)
        - [Convenience Script](#convenience-script)
    - [Development inside the Docker Container](#development-inside-the-docker-container)
      - [Scripts](#scripts)


## About Containers

Containers are a way to package software in a format that can run isolated on a shared operating system. Unlike virtual machines, containers do not bundle a full operating system - only libraries and settings required to make the software work are needed. This makes for efficient, lightweight, self-contained systems and guarantees that software will always run the same, regardless of where it’s deployed.

Throught this course we will be using containers that have all the necessary tools and libraries installed and configured for you. This will allow you to focus on the course content and not on the installation and configuration of the tools. For this course we will be using [Docker](https://www.docker.com/).

Please follow the guide in the Moodle course to install Docker on your computer and to get familiar with it read through the Introduction section of this website: [https://docker-curriculum.com/#introduction](https://docker-curriculum.com/#introduction).



## The Docker Container

We will be using a docker container for this course. This container will have all the necessary tools and libraries installed and configured for you. We provided for you a number of different containers based on different graphic platforms. Please choose the one that best suits your needs.

You can find the container images [here](https://hub.docker.com/repository/docker/georgno/fhtw-ros/general) and the tags follow the following convention:

```
georgno/fhtw-ros:${ROS2-VERSION}-{ROS-VERSION}-ros_{UI}-{GRAPHICS-PLATFORM}
```
, where:
* __ROS2-VERSION__: The ROS2 distribution that you want to use. (humble, iron)
* __ROS-BASE__: The ROS2 base we built upon (desktop, base)
* __UI__: The UI that you want to use (terminal, vnc)
  * Terminal only: This container only has a terminal and you will have to use the command line to do everything.   
    This is the recommended container for this course as it will help you learn to interact with the command line and further because it less computationally intensive.
  * VNC: This container has a full desktop environment and you can use it as if it was a normal computer.   
    This container is recommended if you want to have a fully fledged GUI and you have a powerful computer.
* __GRAPHICS-PLATFORM__: The graphics platform that you want to use (standard, nvidia, amd)
  * Standard__: Use this if you have no decitated graphics card or if you are using a Mac.
  * Nvidia: Use this if you have an Nvidia graphics card.
  * AMD: Use this if you have an AMD graphics card.

> For example, if you want to use ROS2 humble with the desktop base and a terminal UI on a computer with an Nvidia graphics card, you would use the following tag:

```shell
georgno/fhtw-ros:huble-desktop-ros_terminal-nvidia
```


### Starting the Docker Container
There are two ways to start the docker container, both for Windows and Linux. Either you can use a version that will start a terminal only session or a session that will provide a Ubuntu-Mate desktop environment via VNC in the browser.

Both of these versions are available on the docker hub and can be started using either a convenience [script](#convenience-script) or by launching a [devcontainer](#devcontainer).


#### Devcontainer (recommended)

The devcontainer is a docker container that is configured to be used with Visual Studio Code. It will automatically install all necessary extensions and configure the workspace. 

0. Install the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension for Visual Studio Code using either the UI or `code --install-extension ms-vscode-remote.vscode-remote-extensionpack`.
1. Navigate to the folder where you downloaded this github repository and open the folder with VSCode.
2. Open the devcontainer using either:
    - The command palette (F1) and select `Dev Containers: Reopen in Container`
    - The blue button in the bottom left corner of the VSCode window and select `Reopen in Container`


#### Convenience Script (only recommended if you know your way around the terminal)
There is one [powershell script](https://github.com/TW-Robotics/ros2_docker/blob/main/scripts/run_docker.ps1) for Windows and one [bash script](https://github.com/TW-Robotics/ros2_docker/blob/main/scripts/run_docker.sh) for Linux that will start the docker container. The scripts will automatically pull the latest version of the docker container from the docker hub and start it. If you want to build the docker container from source, see the [scripts](#scripts) section below.

##### Windows

0. Make sure the execution policy is set to unrestricted (see [here](https://docs.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_execution_policies)):  
```powershell
Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser
```
1. Start VcXsrv (XLaunch) with following configuration:
![VcXsrv Configuration](./imgs/XmingConfig.PNG)
2. Double klick on [run_docker.ps1](./scripts/run_docker.ps1) 
3. If you are asked to allow docker to mount a folder, click on "Share it" and then on "Apply & Restart". This will allow you to save your projects on your host computer and execute them in the docker container.


##### Linux

0. Make sure that the docker daemon is running:  
```bash
sudo systemctl start docker
```
1. Double klick on [run_docker.sh](./scripts/run_docker.sh)


#### Switch between terminal/desktop, nvidia/amd/standard environment

##### Devcontainer
Adapt the .devcontainer/devcontainer.json file to your needs. The following settings are available:
* ROS_DISTRO: The ROS2 distribution that you want to use. (humble, iron)
* INSTALL_PACKAGE: The ROS2 base we built upon (desktop, base)
* TARGET: The UI that you want to use (terminal, vnc)
* GRAPHICS_PLATFORM: The graphics platform that you want to use (standard, nvidia, amd)
  

##### Convenience Script
The following arguments are available:

```shell
./run_docker.sh -d [iron, humble] -p [desktop, base] -t [terminal, vnc] -g [standard, nvidia, amd]
```

* -d: The ROS2 distribution that you want to use. (humble, iron)
* -p: The ROS2 base we built upon (desktop, base)
* -t: The UI that you want to use (terminal, vnc)
* -g: The graphics platform that you want to use (standard, nvidia, amd)

<br/>
<br/>

### Development inside the Docker Container

The [ros2_ws/src](./ros2_ws/src) folder is mounted to the docker container at __$HOME/ros2_ws/src__. This means that this is the only folder with preserved data once a container is stopped and removed. If you want to save your work, make sure to save it inside this folder.

If you use the conveience script to start the container we still recommend to use to use Visual Studio Code and the following plugins:

- Remote Development (ms-vscode-remote.vscode-remote-extensionpack)  # Enables connecting to a container

After attaching to the continaer you can install the following plugins:
- ROS (ms-iot.vscode-ros)
- C++ Intellisense (austin.code-gnu-global)
- Python (ms-python.python)
- Jupyter (ms-toolsai.jupyter)

#### Scripts
The [scripts](./scripts/) folder contains script to build, run and attach to the docker container. The scripts are only tested on Windows and Linux.

- [run_docker.sh](./scripts/run_docker.sh)/[run_docker.ps1](./scripts/run_docker.ps1) - Runs the docker container
  - Usage: `./run_docker.sh -d [iron, humble] -p [desktop, base] -t [terminal, vnc] -g [standard, nvidia, amd]`
- [attach_terminal.sh](./scripts/attach_terminal.sh)/[attach_terminal.ps1](./scripts/attach_terminal.ps1) - Attaches to a running docker container
  - Usage: `./attach_terminal.sh`   
  Note: This script will attach to the docker container and start a TMUX session to work with multiple terminals inside docker. We recommend familiarising with tmux using the following link: [https://thoughtbot.com/blog/a-tmux-crash-course](https://thoughtbot.com/blog/a-tmux-crash-course)
- [build.sh](./scripts/build.sh)/[build.ps1](./scripts/build.ps1) - Builds the docker container from source and tags it like: `fhtw:${ROS_DISTRO}-${INSTALL_PACKAGE}-${TARGET}-${GRAPHICS_PLATFORM}`
  - Usage: `./build.sh -d [iron, humble] -p [desktop, base] -t [ros_terminal, ros_vnc]  -g [standard, nvidia, amd]`