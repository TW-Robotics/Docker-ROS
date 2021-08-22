FROM ros:melodic
LABEL maintainer = "Georg Novotny FHTW"
RUN apt update && \
    apt install -y\
    less htop nmon tmux gdb gosu python-pip python3-pip\
    sudo vim libgl1-mesa-glx libgl1-mesa-dri git xterm curl\
    iproute2 iputils-ping synaptic bash-completion libboost-all-dev clang-format bc\
    imagemagick psmisc protobuf-compiler ros-melodic-dwa-local-planner\
    ros-melodic-costmap-2d ros-melodic-hector-gazebo* ros-melodic-global-planner\
    ros-melodic-turtlebot3* ros-melodic-navigation ros-melodic-pid\
    ros-melodic-rosdoc-lite ros-melodic-gmapping\
    ros-melodic-rqt* ros-melodic-urdf* ros-melodic-tf2* ros-melodic-roslint --no-install-recommends \
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
    cd .. &&\
    /ros_entrypoint.sh catkin_make
RUN chown $USERNAME:$USERNAME --recursive /home/$USERNAME/catkin_ws
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/catkin_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc
RUN pip3 install setuptools wheel 
RUN pip3 install gdbgui jupyter

COPY ./docker_install /home/$USERNAME/docker_install
RUN bash /home/$USERNAME/docker_install/install_vim.sh "${USERNAME}"
RUN rm -rf /home/$USERNAME/docker_install

RUN echo 'if [ -z "$TMUX" ]; then     tmux attach -t default || tmux new -s default; fi' >> /home/fhtw_user/.bashrc
   

# OpenCV 3.4.0
RUN  apt update && apt install -y checkinstall python-dev python-numpy libtbb2 \
        libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev cmake git pkg-config \
        libavcodec-dev libavformat-dev libswscale-dev cmake git libgtk2.0-dev pkg-config libavcodec-dev \
        libavformat-dev libswscale-dev libopencv-dev build-essential checkinstall  cmake pkg-config \
        yasm libjpeg-dev libswscale-dev libdc1394-22-dev libxine2-dev  libv4l-dev python-dev python-numpy \
        libtbb-dev  qtbase5-dev  libgtk2.0-dev libfaac-dev libmp3lame-dev  libopencore-amrnb-dev \
        libopencore-amrwb-dev libtheora-dev  libvorbis-dev libxvidcore-dev x264 v4l-utils ffmpeg \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git  && git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && git fetch -a && git checkout 3.4.0 && mkdir build && cd ../opencv_contrib &&  git fetch -a && git checkout 3.4.0 && cd ../opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE\
        -D CMAKE_INSTALL_PREFIX=/usr/local\
        -D OPENCV_ENABLE_NONFREE:BOOL=ON\
        -D OPENCV_EXTRA_MODULES_PATH="/tmp/opencv_contrib/modules" \
        -D BUILD_opencv_aruco=OFF -D BUILD_opencv_python2=OFF\
        -D WITH_IPP=ON -D WITH_TBB=ON\
        -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=OFF\
        -D BUILD_opencv_cudacodec=OFF\
        -D BUILD_opencv_python=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF\
        -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_OPENCL=ON -D WITH_GSTREAMER=ON .. && \
    cores=$(grep -c processor < /proc/cpuinfo) && if [ "$cores" -gt "1" ]; then cores=$((cores-1)); fi && \
    make -j"$cores" && \
    sudo checkinstall -y --pkgname "opencv3.4.0" && \
    sudo apt install -f && \
    sudo sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf' && \
    sudo ldconfig && \
    make clean && \
    pkgPrefix=$(grep -E "^prefix" < /usr/local/lib/pkgconfig/opencv.pc | tr '=' ' ' | awk '{print $2}') && \
    if [ -d "$pkgPrefix/share/OpenCV/3rdparty/lib" ]; then sudo sed -i -e 's/-L${exec_prefix}\/lib/& -L${exec_prefix}\/share\/OpenCV\/3rdparty\/lib/g' /usr/local/lib/pkgconfig/opencv.pc; fi && \
    echo "OpenCV 3.4.0 Installed"
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
