ARG GRAPHICS_PLATFORM=standard

# Create base images based on GPU acceleration
## Nvidia-based container
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 as build_nvidia
ONBUILD ARG NVIDIA_VISIBLE_DEVICES=all
ONBUILD ARG NVIDIA_DRIVER_CAPABILITIES=graphics,utility
ONBUILD ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES}
ONBUILD ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES}
ONBUILD ENV QT_X11_NO_MITSHM 1
ONBUILD ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64:/usr/local/cuda/lib64/
ONBUILD ENV PATH=${PATH}:/usr/local/cuda/bin
## Intel OneAPI Container
FROM intel/oneapi-aikit:2023.1.0-devel-ubuntu22.04 as build_intel

## AMD ROCm Container
FROM rocm/dev-ubuntu-22.04:latest as build_amd

## Generic container with MESA
FROM ubuntu:jammy as build_standard

## Generic container for WSL
FROM ubuntu:jammy as build_wsl
# Set LD library path as in https://github.com/microsoft/wslg/tree/main/samples/container
ONBUILD ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib

#############################################################
##########          REAL BUILD STARTS HERE         ##########

## Build container from specified source
FROM build_${GRAPHICS_PLATFORM} as build_base
LABEL image.source="https://github.com/SimonSchwaiger/ros-ml-container"

ENV DEBIAN_FRONTEND="noninteractive"

# Install mesa for GUI
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgl1-mesa-glx libgl1-mesa-dri
LABEL maintainer = "Georg Novotny FHTW"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Install some basic packages
RUN apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
    wget curl git sudo lsb-release locales bash-completion tzdata gosu tmux xterm less bc \
        build-essential neovim \
        python3-pip --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*


###############
# Install ROS #
FROM build_base as build_ros
ARG ROS_DISTRO=iron
ARG BASE_PACKAGE=desktop

ENV DEBIAN_FRONTEND="noninteractive"
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo "--------------------------------------------------------------------------------------------------------------"
RUN echo "Installing ROS2 ${ROS_DISTRO}-${BASE_PACKAGE}"
RUN echo "--------------------------------------------------------------------------------------------------------------"
RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo jammy) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && apt-get upgrade -y && \
    apt-get install -y ros-dev-tools ros-${ROS_DISTRO}-${BASE_PACKAGE} ros-${ROS_DISTRO}-rqt* \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool --no-install-recommends && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*


#####################################
# Add User and install ROS Packages #
FROM build_ros as ros_terminal
ENV DEBIAN_FRONTEND="noninteractive"
ENV USERNAME fhtw_user
ARG USER_ID=1000
ARG GROUP_ID=1000

ARG ROS_DISTRO=iron
ARG BASE_PACKAGE=desktop
ENV ROS_WORKSPACE /home/${USERNAME}/ros2_ws
ENV COLCON_HOME=/home/${USERNAME}/.colcon

# Create user
RUN groupadd --gid $GROUP_ID ${USERNAME} && \
        useradd --gid $GROUP_ID -m ${USERNAME} && \
        echo "${USERNAME}:${USERNAME}" | chpasswd && \
        usermod --shell /bin/bash ${USERNAME} && \
        usermod -aG sudo ${USERNAME} && \
        usermod  --uid $USER_ID ${USERNAME} && \
        echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} && \
        chmod 0440 /etc/sudoers.d/${USERNAME}

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]


# Install ROS packages
RUN apt-get update -q && apt-get install -y ros-${ROS_DISTRO}-nav2* ros-${ROS_DISTRO}-turtlebot3* --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*


RUN su ${USERNAME} -c "rosdep update"; \
    echo "export PATH=/home/${USERNAME}/.local/bin:${PATH}" && \
    echo "export TURTLEBOT3_MODEL=burger" >> /home/${USERNAME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc && \
	source "/usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source ${ROS_WORKSPACE}/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    mkdir -p ${ROS_WORKSPACE}/src

RUN git clone https://github.com/osrf/gazebo_models/ /home/${USERNAME}/.gazebo/models && \
    mkdir -p /home/${USERNAME}/sim_ws/src && \
    git clone https://github.com/NovoG93/sjtu_drone /home/${USERNAME}/sim_ws/src/sjtu_drone && \
    git clone https://github.com/NovoG93/robotino /home/${USERNAME}/sim_ws/src/robotino && \
    sed -i "s#source /opt/ros/${ROS_DISTRO}/setup.bash#source /opt/ros/${ROS_DISTRO}/setup.bash\nsource /home/${USERNAME}/sim_ws/install/setup.bash#g" /home/${USERNAME}/.bashrc 
 
# Initialize and update rosdep as the user
RUN su ${USERNAME} -c "rosdep update"
 
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash  && colcon build --symlink-install" && \
    chown ${USERNAME}:${USERNAME} --recursive /home/${USERNAME}/ros2_ws && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/sim_ws/
RUN chown ${USERNAME}:${USERNAME} --recursive /home/${USERNAME}/sim_ws && \
    su ${USERNAME} -c "source /opt/ros/${ROS_DISTRO}/setup.bash && whoami && sudo apt-get update -q && rosdep install --from-path src --ignore-src -r -y" && \
    su ${USERNAME} -c "source /opt/ros/${ROS_DISTRO}/setup.bash &&  colcon build --symlink-install --continue-on-error"
    
RUN mkdir -p /tmp/docker_install
COPY ./docker_install/install_vim.sh /tmp/docker_install/install_vim.sh
COPY ./docker_install/tmux/.tmux.conf /home/${USERNAME}/.tmux.conf
RUN su ${USERNAME} -c "bash /tmp/docker_install/install_vim.sh ${USERNAME}"
COPY ./docker_install/requirements.txt /tmp/docker_install/requirements.txt
RUN sudo -u ${USERNAME} pip install -r /tmp/docker_install/requirements.txt
ENV PATH ${PATH}:/home/${USERNAME}/.local/bin
COPY ./docker_install/colcon_defaults.yaml /home/${USERNAME}/.colcon/defaults.yaml
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.colcon/defaults.yaml
# SET TMUX as default shell if current shell not opened by VSCODE
RUN su ${USERNAME} -c "echo 'if [ -z \"\$TMUX\" ] && [ \"\$TERM_PROGRAM\" != \"vscode\" ] && [ -z \"\$SESSION_MANAGER\" ]; then tmux attach -t default || tmux new -s default; fi' >> /home/${USERNAME}/.bashrc"
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}


ENV USER ${USERNAME}
ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=
ENV SHELL=/bin/bash

WORKDIR ${ROS_WORKSPACE}/
COPY ./docker_install/terminal_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && rm -rf /tmp/docker_install
RUN sed -i "s/DEFALTUSER/${USERNAME}/g" /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]


##############################################
# Install Ubuntu Mate desktop and VNC server #
FROM ros_terminal as ros_vnc
ARG VNC_PASSWORD=123456
ENV HOME=/home/${USERNAME}
ENV DEBIAN_FRONTEND="noninteractive"
RUN apt-get update -q && \
    apt-get install -y \
        ubuntu-mate-desktop \
        tigervnc-standalone-server tigervnc-common tini \
        supervisor terminator && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*


# Install noVNC and websockify
RUN git clone https://github.com/novnc/noVNC.git /usr/lib/novnc && \
    pip3 install websockify && \
    ln -s /usr/lib/novnc/vnc.html /usr/lib/novnc/index.html

# Configure VNC
RUN mkdir -p /home/${USERNAME}/.vnc && \
    touch /home/${USERNAME}/.Xauthority && \
    echo "${VNC_PASSWORD}" | vncpasswd -f > /home/${USERNAME}/.vnc/passwd && \
    chmod 600 /home/${USERNAME}/.vnc/passwd && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.vnc && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.Xauthority && \
    sed -i "s/password = WebUtil.getConfigVar('password');/password = '$VNC_PASSWORD'/" /usr/lib/novnc/app/ui.js

# xstartup
ENV XSTARTUP_PATH /home/${USERNAME}/.vnc/xstartup
COPY ./docker_install/vnc/xstartup ${XSTARTUP_PATH}
RUN chmod 755 /home/${USERNAME}/.vnc/xstartup

# vncserver launch
ENV VNCRUN_PATH /home/${USERNAME}/.vnc/vnc_run.sh
COPY ./docker_install/vnc/vnc_run.sh ${VNCRUN_PATH}
RUN sed -i "s/'\$USER'/${USERNAME}/g" ${VNCRUN_PATH} && \
    chmod +x /home/${USERNAME}/.vnc/vnc_run.sh

# Supervisor
ENV SUPERVISOR_CONF_PATH /etc/supervisor/conf.d/supervisord.conf
COPY ./docker_install/vnc/supervisord.conf ${SUPERVISOR_CONF_PATH}
RUN sed -i "s#'\$USER'#${USERNAME}#g" ${SUPERVISOR_CONF_PATH} && \
    sed -i "s#'\$VNCRUN_PATH'#${VNCRUN_PATH}#g" ${SUPERVISOR_CONF_PATH}

# Expose VNC and noVNC ports
EXPOSE 5901 6901
# Expose Websockify port
EXPOSE 80 6080

# Mate configuration
# Disable auto update and crash report
RUN sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades && \
    sed -i 's/enabled=1/enabled=0/g' /etc/default/apport && \
    gsettings set org.gnome.desktop.default-applications.terminal exec /usr/bin/terminator && \
    gsettings set org.gnome.desktop.default-applications.terminal exec-arg "-x" && \
    sudo update-alternatives --install /usr/bin/x-terminal-emulator x-terminal-emulator /usr/bin/terminator 1
# Enable apt-get completion
RUN rm /etc/apt/apt.conf.d/docker-clean

ENV DISPLAY :1
ENV DEBIAN_FRONTEND=
# Copy entrypoint script
COPY ./docker_install/vnc_entrypoint.sh /entrypoint.sh
RUN sed -i "s/DEFALTUSER/${USERNAME}/g" /entrypoint.sh && \
    sed -i "s#-n ##g" /entrypoint.sh && \
    chmod +x /entrypoint.sh

ARG GRAPHICS_PLATFORM=standard
RUN if [ "$GRAPHICS_PLATFORM" = "nvidia" ]; then \
        wget https://downloads.sourceforge.net/project/virtualgl/3.1/virtualgl_3.1_amd64.deb && \
        apt-get update -q && apt-get install ./virtualgl_*.deb -y && \
        rm virtualgl_*.deb && \
        /opt/VirtualGL/bin/vglserver_config -config +s +f -t; \
    fi
ENV VGL_DISPLAY ${DISPLAY}
ENV VGL_LOGO 1
ENV VGL_VERBOSE 1


RUN chown -R ${USER}:${USER} /home/${USER}
ENTRYPOINT ["/entrypoint.sh"]