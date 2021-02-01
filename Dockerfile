FROM ros:noetic
LABEL maintainer = "Georg Novotny FHTW"

RUN apt update && \
    apt-get install -y bash-completion\
    less htop tmux gosu python3-pip git\
    ros-noetic-amcl ros-noetic-angles ros-noetic-base-local-planner ros-noetic-clear-costmap-recovery \
    ros-noetic-costmap-2d ros-noetic-diagnostic-updater ros-noetic-hls-lfcd-lds-driver ros-noetic-interactive-markers \
    ros-noetic-joint-state-publisher ros-noetic-kdl-parser ros-noetic-laser-geometry ros-noetic-map-msgs \
    ros-noetic-map-server ros-noetic-move-base ros-noetic-move-base-msgs ros-noetic-nav-core ros-noetic-navfn \ 
    ros-noetic-robot-state-publisher ros-noetic-rotate-recovery \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-geometry-msgs ros-noetic-tf2-kdl ros-noetic-tf2-msgs ros-noetic-tf2-py \
    ros-noetic-tf2-ros ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-example ros-noetic-turtlebot3-navigation ros-noetic-turtlebot3-slam \
    ros-noetic-turtlebot3-teleop ros-noetic-urdf ros-noetic-voxel-grid ros-noetic-xacro \
    ros-noetic-rosdoc-lite ros-noetic-gmapping ros-noetic-rqt* ros-noetic-gazebo-ros ros-noetic-turtlebot3-simulations --no-install-recommends\
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
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc
RUN pip3 install gdbgui

# RUN sudo apt update && sudo apt install -y checkinstall python-dev python-numpy libtbb2 \
#     libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev cmake git pkg-config \
#     libavcodec-dev libavformat-dev libswscale-dev cmake git libgtk2.0-dev pkg-config libavcodec-dev \
#     libavformat-dev libswscale-dev libopencv-dev build-essential checkinstall  cmake pkg-config \
#     yasm libjpeg-dev libswscale-dev libdc1394-22-dev libxine2-dev  libv4l-dev python-dev python-numpy \
#     libtbb-dev  qtbase5-dev  libgtk2.0-dev libfaac-dev libmp3lame-dev  libopencore-amrnb-dev \
#     libopencore-amrwb-dev libtheora-dev  libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg \
#     libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev

# RUN cd /tmp && git clone https://github.com/opencv/opencv.git  && git clone https://github.com/opencv/opencv_contrib.git && \
#     cd opencv && git fetch -a && git checkout 3.4.13 && mkdir build && cd ../opencv_contrib &&  git fetch -a && git checkout 3.4.13 && cd ../opencv/build && \
#     cmake -D CMAKE_BUILD_TYPE=RELEASE\
#         -D CMAKE_INSTALL_PREFIX=/usr/local\
#         -D OPENCV_ENABLE_NONFREE:BOOL=ON\
#         -D OPENCV_EXTRA_MODULES_PATH="/tmp/opencv_contrib/modules" \
#         -D BUILD_opencv_aruco=OFF -D BUILD_opencv_python2=OFF\
#         -D WITH_IPP=ON -D WITH_TBB=ON\
#         -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=OFF\
#         -D BUILD_opencv_cudacodec=OFF\
#         -D BUILD_opencv_python=OFF \
#         -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF\
#         -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_OPENCL=ON -D WITH_GSTREAMER=ON .. && \
#     cores=$(grep -c processor < /proc/cpuinfo) && if [ "$cores" -gt "1" ]; then cores=$((cores-1)); fi && \
#     make -j"$cores" && \
#     sudo checkinstall -y --pkgname "opencv3.4.13" && \
#     sudo apt install -f && \
#     sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf' && \
#     sudo ldconfig && \
#     make clean && \
#     pkgPrefix=$(grep -E "^prefix" < /usr/local/lib/pkgconfig/opencv.pc | tr '=' ' ' | awk '{print $2}') && \
#     if [ -d "$pkgPrefix/share/OpenCV/3rdparty/lib" ]; then sudo sed -i -e 's/-L${exec_prefix}\/lib/& -L${exec_prefix}\/share\/OpenCV\/3rdparty\/lib/g' /usr/local/lib/pkgconfig/opencv.pc; fi && \
#     echo "OpenCV 3.4.13 Installed"


COPY ./docker_install /home/$USERNAME/docker_install
RUN bash /home/$USERNAME/docker_install/install_vim.sh "${USERNAME}"
RUN rm -rf /home/$USERNAME/docker_install

RUN echo 'if [ -z "$TMUX" ]; then tmux attach -t default || tmux new -s default; fi' >> /home/fhtw_user/.bashrc
   
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

#-----------#
# CUDA Base #
#-----------#
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --autoremove -y curl \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.0.3

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-0=11.0.221-1 \
    cuda-compat-11-0 \
    && ln -s cuda-11.0 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*


ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.0 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 brand=tesla,driver>=450,driver<451"
#--------------#
# CUDA runtime #
#--------------#
ENV NCCL_VERSION 2.8.3

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-11-0=11.0.3-1 \
    libnpp-11-0=11.1.0.245-1 \
    cuda-nvtx-11-0=11.0.167-1 \
    libcublas-11-0=11.2.0.252-1 \
    libnccl2=$NCCL_VERSION-1+cuda11.0 \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold libcublas-11-0 libnccl2

# CUDNN #
ENV CUDNN_VERSION 8.0.5.39
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=$CUDNN_VERSION-1+cuda11.0 \
    && apt-mark hold libcudnn8 && \
    rm -rf /var/lib/apt/lists/*


#------------#
# CUDA devel #
#------------#
ENV NCCL_VERSION 2.8.3

RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinfo5 libncursesw5 \
    cuda-cudart-dev-11-0=11.0.221-1 \
    cuda-command-line-tools-11-0=11.0.3-1 \
    cuda-minimal-build-11-0=11.0.3-1 \
    cuda-libraries-dev-11-0=11.0.3-1 \
    cuda-nvml-dev-11-0=11.0.167-1 \
    libnpp-dev-11-0=11.1.0.245-1 \
    libnccl-dev=2.8.3-1+cuda11.0 \
    libcublas-dev-11-0=11.2.0.252-1 \
    libcusparse-11-0=11.1.1.245-1 \
    libcusparse-dev-11-0=11.1.1.245-1 \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold libcublas-dev-11-0 libnccl-dev
ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

# CUDNN #
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=$CUDNN_VERSION-1+cuda11.0 \
    libcudnn8-dev=$CUDNN_VERSION-1+cuda11.0 \
    && apt-mark hold libcudnn8 && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install tensorflow keras wandb torch


ENTRYPOINT [ "/ros_entrypoint.sh" ]
