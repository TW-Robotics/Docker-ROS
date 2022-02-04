#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/root/.Xauthority
SHARED_DIR=/home/fhtw_user/catkin_ws/src/fhtw
HOST_DIR=$(pwd)/src

echo -e "\e[32mMounting fodler:
    $HOST_DIR    to
    $SHARED_DIR\e[0m"


docker run \
    -it --rm \
    --name fhtw-ros \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    --runtime nvidia \
    --gpus all \
    fhtw/ros-melodic-gpu:latest bash
