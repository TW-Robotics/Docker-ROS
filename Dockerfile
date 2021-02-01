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
    ros-melodic-rqt* ros-melodic-urdf* \
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
RUN pip3 install gdbgui

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
    apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130
ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-0 \
    && ln -s cuda-10.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=410,driver<411"

#--------------#
# CUDA runtime #
#--------------#
ENV NCCL_VERSION 2.6.4

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-$CUDA_PKG_VERSION \
    cuda-npp-$CUDA_PKG_VERSION \
    cuda-nvtx-$CUDA_PKG_VERSION \
    cuda-cublas-10-0=10.0.130-1 \
    libnccl2=$NCCL_VERSION-1+cuda10.0 \
    && apt-mark hold libnccl2 \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold cuda-cublas-10-0


# CUDNN #
ENV CUDNN_VERSION 7.6.5.32
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
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
    cuda-cublas-dev-10-0=10.0.130-1 \
    libnccl-dev=2.6.4-1+cuda10.0 \
    && apt-mark hold libnccl-dev \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold cuda-cublas-dev-10-0

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs


# CUDNN #
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=$CUDNN_VERSION-1+cuda10.0 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 \
    && apt-mark hold libcudnn7 && \
    rm -rf /var/lib/apt/lists/*


RUN pip install tensorflow-gpu torch torchvision future 

RUN pip3 install jupyter ipykernel

RUN pip install ipykernel

ENTRYPOINT [ "/ros_entrypoint.sh" ]
