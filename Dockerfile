FROM ros:melodic
LABEL maintainer = "Georg Novotny FHTW"
RUN apt update && \
    apt install -y\
    less htop nmon tmux gdb gosu python-pip python3-pip vim\
    sudo git xterm curl\
    iproute2 iputils-ping synaptic bash-completion libboost-all-dev clang-format bc\
    imagemagick psmisc protobuf-compiler ros-melodic-dwa-local-planner\
    ros-melodic-costmap-2d ros-melodic-hector-gazebo* ros-melodic-global-planner\
    ros-melodic-turtlebot3* ros-melodic-navigation ros-melodic-pid\
    ros-melodic-rosdoc-lite ros-melodic-gmapping \
    ros-melodic-rqt* ros-melodic-urdf* --no-install-recommends \
    && rm -rf /var/lib/apt/lists/

ENV USERNAME fhtw_user
ARG USER_ID=1000
ARG GROUP_ID=15214

RUN groupadd --gid $GROUP_ID $USERNAME && \
        useradd --gid $GROUP_ID -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        usermod  --uid $USER_ID $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME

RUN echo "export TURTLEBOT3_MODEL=burger" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_HOSTNAME=\"\$(hostname -I | awk '{print \$1;}')\"" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_IP=\"\$(hostname -I | awk '{print \$1;}')\"" >> /home/$USERNAME/.bashrc
RUN echo 'echo "ROS_HOSTNAME=>$ROS_HOSTNAME<"' >> /home/$USERNAME/.bashrc
RUN echo 'echo "ROS_IP=>$ROS_IP<"' >> /home/$USERNAME/.bashrc


RUN mkdir -p /home/$USERNAME/catkin_ws/src &&\
    cd /home/$USERNAME/catkin_ws/src &&\
    /ros_entrypoint.sh catkin_init_workspace &&\
    git clone https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/ openai-ros && \
    git clone https://bitbucket.org/theconstructcore/openai_examples_projects.git && \
    cd .. &&\
    /ros_entrypoint.sh catkin_make
RUN chown $USERNAME:$USERNAME --recursive /home/$USERNAME/catkin_ws
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./docker_install /home/$USERNAME/docker_install
RUN bash /home/$USERNAME/docker_install/install_vim.sh "${USERNAME}"
RUN rm -rf /home/$USERNAME/docker_install

RUN echo 'if [ -z "$TMUX" ]; then     tmux attach -t default || tmux new -s default; fi' >> /home/fhtw_user/.bashrc
   
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

#-----------#
# CUDA Base #
#-----------#
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.2.89
ENV CUDA_PKG_VERSION 10-2=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-2 \
    && ln -s cuda-10.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.2 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441"


#--------------#
# CUDA runtime #
#--------------#
ENV NCCL_VERSION 2.8.3

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-$CUDA_PKG_VERSION \
    cuda-npp-$CUDA_PKG_VERSION \
    cuda-nvtx-$CUDA_PKG_VERSION \
    libcublas10=10.2.2.89-1 \
    libnccl2=$NCCL_VERSION-1+cuda10.2 \
    && apt-mark hold libnccl2 \
    && rm -rf /var/lib/apt/lists/*



# CUDNN #
ENV CUDNN_VERSION 7.6.5.32
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.2 \
    && apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*



#------------#
# CUDA devel #
#------------#
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-nvml-dev-$CUDA_PKG_VERSION \
    cuda-command-line-tools-$CUDA_PKG_VERSION \
    cuda-nvprof-$CUDA_PKG_VERSION \
    cuda-npp-dev-$CUDA_PKG_VERSION \
    cuda-libraries-dev-$CUDA_PKG_VERSION \
    cuda-minimal-build-$CUDA_PKG_VERSION \
    libcublas-dev=10.2.2.89-1 \
    libnccl-dev=2.8.3-1+cuda10.2 \
    && apt-mark hold libnccl-dev \
    && rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs



# CUDNN #
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.2 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.2 \
    && apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*

RUN sudo apt update && sudo apt install -y python-setuptools python3-setuptools && rm -rf /var/lib/apt/lists/
RUN pip install wheel && pip3 install wheel
RUN pip3 install gdbgui tensorflow tensorboard pytorch torchvision gym

ENTRYPOINT [ "/ros_entrypoint.sh" ]
